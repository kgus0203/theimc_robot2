#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fairino_waypoint_test/action/scan_side.hpp>
#include <fairino_waypoint_test/srv/capture_waypoint.hpp>
#include "recorded_poses.hpp"
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;
using Marker = visualization_msgs::msg::Marker;
using Joints = std::vector<double>;
constexpr double RAD = 3.14159265358979323846 / 180.0;
const std::vector<std::string> NAMES{"j1", "j2", "j3", "j4", "j5", "j6"};
const Joints ZERO{0, -90, 0, -90, 0, 0};
const Joints RIGHT{-90, -50, -100, -23, 0, 0};
const Joints LEFT{90, -140, 100, -140, 0, 0};

// Check availability before constructing MoveGroupInterface: its constructor starts
// internal threads and otherwise waits indefinitely for missing action servers.
bool waitForMoveIt(const rclcpp::Node::SharedPtr& node)
{
  const double timeout = node->get_parameter("connection_timeout").as_double();
  if (!std::isfinite(timeout) || timeout <= 0)
    throw std::runtime_error("connection_timeout must be positive and finite");
  auto planning = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node, "/move_action");
  auto execution = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
    node, "/execute_trajectory");
  RCLCPP_INFO(node->get_logger(), "Waiting up to %.1f seconds for MoveIt action servers...", timeout);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while (rclcpp::ok()) {
    if (planning->action_server_is_ready() && execution->action_server_is_ready()) return true;
    if (std::chrono::steady_clock::now() >= deadline)
      throw std::runtime_error(
        "MoveIt action servers unavailable. Start demo.launch.py first, or launch this test "
        "with start_demo:=true when no demo is running. If demo is already running, "
        "check ROS_DOMAIN_ID and ROS discovery settings in both terminals.");
    std::this_thread::sleep_for(100ms);
  }
  return false;
}

std::map<std::string, double> radians(const Joints& degrees)
{
  std::map<std::string, double> result;
  for (size_t i = 0; i < NAMES.size(); ++i) result[NAMES[i]] = degrees.at(i) * RAD;
  return result;
}

class Test
{
  using Scan = fairino_waypoint_test::action::ScanSide;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Scan>;
public:
  explicit Test(const rclcpp::Node::SharedPtr& node)
  : node_(node), group_(node, "fairino5_v6_group", nullptr,
      rclcpp::Duration::from_seconds(node->get_parameter("connection_timeout").as_double())),
    state_(group_.getRobotModel())
  {
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/arm/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        measured_joints_.clear();
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
          measured_joints_[msg->name[i]] = msg->position[i];
        joint_received_at_ = std::chrono::steady_clock::now();
        ++joint_sequence_;
        joint_updated_.notify_all();
      });
    const double speed = node->get_parameter("velocity_scale").as_double();
    const double accel = node->get_parameter("acceleration_scale").as_double();
    waypoint_limit_ = node->get_parameter("waypoint_limit").as_int();
    if (!std::isfinite(speed) || speed <= 0 || speed > 1 ||
        !std::isfinite(accel) || accel <= 0 || accel > 1)
      throw std::runtime_error("Require velocity/acceleration scales in (0,1]");
    if (waypoint_limit_ < 0 || waypoint_limit_ > 9)
      throw std::runtime_error("waypoint_limit must be in [0,9] (0 means all)");
    group_.setMaxVelocityScalingFactor(speed);
    group_.setMaxAccelerationScalingFactor(accel);
    group_.setPlanningPipelineId("ompl");
    // This config exposes RRTConnect as the group default (no named planner entry).
    group_.setPlannerId("");
    group_.setPlanningTime(10.0);
    group_.setNumPlanningAttempts(5);
    state_.setToDefaultValues();
    state_.setVariablePositions(radians(ZERO));
    state_.update();
    reference_ = state_.getGlobalLinkTransform("shoulder_link");
    pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/waypoint_test/markers", rclcpp::QoS(1).transient_local());
    path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/waypoint_test/planned_path", rclcpp::QoS(1).transient_local());
    trajectory_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).transient_local());
    label("front", {0, -0.65, 0.15}, 0);
    label("right (-x)", {-0.65, 0, 0.15}, 1);
    label("left (+x)", {0.65, 0, 0.15}, 2);
    pub_->publish(markers_);
  }

  ~Test()
  {
    if (worker_.joinable()) worker_.join();
  }

  void startServer()
  {
    addObstacles();
    server_ = rclcpp_action::create_server<Scan>(node_, "/scan_side",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Scan::Goal> goal) {
        if (goal->side != "right" && goal->side != "left") {
          RCLCPP_WARN(node_->get_logger(), "Rejected side: use right or left");
          return rclcpp_action::GoalResponse::REJECT;
        }
        bool idle = false;
        if (!busy_.compare_exchange_strong(idle, true)) {
          RCLCPP_WARN(node_->get_logger(), "Rejected request: another scan is active");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        // Cancellation takes effect after the current MoveIt operation finishes.
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        if (worker_.joinable()) worker_.join();
        worker_ = std::thread([this, handle] { serve(handle); });
      });
    RCLCPP_INFO(node_->get_logger(), "READY: /scan_side action waiting for side=right or left");
  }

private:
  void addObstacles()
  {
    const double distance = node_->get_parameter("wall_distance").as_double();
    const double thickness = node_->get_parameter("wall_thickness").as_double();
    const double width = node_->get_parameter("wall_width").as_double();
    const double height = node_->get_parameter("wall_height").as_double();
    for (double value : {distance, thickness, width, height})
      if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error("Wall dimensions and distance must be finite and positive (metres)");
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene.is_diff = true;
    request->scene.robot_state.is_diff = true;
    auto box = [&](const std::string& id, const std::string& frame,
                   const Eigen::Isometry3d& pose, const std::vector<double>& size,
                   float red, float green, float blue) {
      moveit_msgs::msg::CollisionObject object;
      object.id = id;
      object.header.frame_id = frame;
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions.assign(size.begin(), size.end());
      geometry_msgs::msg::Pose position;
      position.position.x = pose.translation().x();
      position.position.y = pose.translation().y();
      position.position.z = pose.translation().z();
      const Eigen::Quaterniond q(pose.rotation());
      position.orientation.x = q.x();
      position.orientation.y = q.y();
      position.orientation.z = q.z();
      position.orientation.w = q.w();
      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(position);
      request->scene.world.collision_objects.push_back(object);
      moveit_msgs::msg::ObjectColor color;
      color.id = id;
      color.color.r = red;
      color.color.g = green;
      color.color.b = blue;
      color.color.a = 0.65f;
      request->scene.object_colors.push_back(color);
    };
    // Distance is to the inner face; thickness extends away from the robot.
    for (bool right : {true, false}) {
      Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
      local.translation() = Eigen::Vector3d(
        (right ? -1 : 1) * (distance + thickness / 2), 0, height / 2);
      box(right ? "scan_right_wall" : "scan_left_wall",
          group_.getRobotModel()->getModelFrame(), reference_ * local,
          {thickness, width, height}, right ? 0.3f : 0.8f, 0.55f, right ? 0.9f : 0.3f);
    }
    Eigen::Isometry3d support = Eigen::Isometry3d::Identity();
    // Base mesh spans z=0..0.079m; shoulder mounting skirt starts at 0.074m.
    // Keep the 440mm cube, raising its top to cover the fixed base.
    support.translation().z() = 0.079 - 0.22;
    box("scan_base_box", "base_link", support, {0.44, 0.44, 0.44}, 0.55f, 0.55f, 0.55f);
    const auto timeout = std::chrono::duration<double>(
      node_->get_parameter("connection_timeout").as_double());
    // These clients are created after the main node has already been added to
    // its executor.  On Humble, the executor can miss those newly-created
    // entities, leaving the futures pending even though MoveIt replied.  Use a
    // short-lived node and spin it explicitly while building the scene.
    auto setup_node = std::make_shared<rclcpp::Node>(
      "fairino_waypoint_scene_setup",
      rclcpp::NodeOptions()
        .start_parameter_services(false)
        .start_parameter_event_publisher(false));
    auto scene_client = setup_node->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
    if (!scene_client->wait_for_service(timeout))
      throw std::runtime_error("Cannot read mounting collision permissions");
    auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    scene_request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto scene_future = scene_client->async_send_request(scene_request);
    if (rclcpp::spin_until_future_complete(setup_node, scene_future, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("Timed out reading mounting collision permissions");
    collision_detection::AllowedCollisionMatrix allowed(
      scene_future.get()->scene.allowed_collision_matrix);
    // Only the fixed base and j1 mounting skirt overlap the support. Preserve
    // all other scene permissions; upperarm and subsequent links still collide.
    allowed.setEntry("scan_base_box", "base_link", true);
    allowed.setEntry("scan_base_box", "shoulder_link", true);
    allowed.getMessage(request->scene.allowed_collision_matrix);
    auto client = setup_node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene");
    if (!client->wait_for_service(timeout))
      throw std::runtime_error("Cannot add obstacles: /apply_planning_scene unavailable");
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(setup_node, future, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS || !future.get()->success)
      throw std::runtime_error("Obstacle registration failed; scan action server will not start");
    RCLCPP_INFO(node_->get_logger(),
      "Collision scene ready: walls at +/-%.3f m (inner faces), base box 0.44 m cube with top at base_link z=0.079 m",
      distance);
  }

  void checkpoint()
  {
    if (!rclcpp::ok()) throw std::runtime_error("Shutdown requested");
    if (goal_->is_canceling()) throw std::runtime_error("Canceled after current MoveIt operation");
  }

  void feedback(const std::string& stage)
  {
    auto message = std::make_shared<Scan::Feedback>();
    message->side = goal_->get_goal()->side;
    message->stage = stage;
    message->completed_waypoints = completed_;
    const size_t available = goal_->get_goal()->side == "right" ? recorded::poses.size() :
      recorded::left_poses.size();
    message->total_waypoints = waypoint_limit_ == 0 ? available :
      std::min<size_t>(waypoint_limit_, available);
    goal_->publish_feedback(message);
  }

  void serve(const std::shared_ptr<GoalHandle>& handle)
  {
    goal_ = handle;
    completed_ = 0;
    auto result = std::make_shared<Scan::Result>();
    try {
      checkpoint();
      capture_client_.reset();
      if (!handle->get_goal()->capture_service.empty()) {
        if (handle->get_goal()->inspection_id.empty())
          throw std::runtime_error("inspection_id is required for capture");
        capture_client_ = node_->create_client<fairino_waypoint_test::srv::CaptureWaypoint>(
          handle->get_goal()->capture_service);
        if (!capture_client_->wait_for_service(5s))
          throw std::runtime_error("Inspection capture service unavailable");
      }
      feedback("checking joint states");
      if (!group_.startStateMonitor(30.0) || !group_.getCurrentState(30.0))
        throw std::runtime_error("No current joint state received");
      checkpoint();
      paths_.markers.clear();
      Marker clear;
      clear.action = Marker::DELETEALL;
      paths_.markers.push_back(clear);
      path_pub_->publish(paths_);
      paths_.markers.clear();
      state_.setVariablePositions(recorded::target(recorded::standby));
      state_.update();
      if (!state_.satisfiesBounds())
        throw std::runtime_error("standby: joint limits exceeded");
      if (handle->get_goal()->side == "right") {
        runRecorded(recorded::poses);
      } else {
        runRecorded(recorded::left_poses);
      }
      checkpoint();
      feedback("returning to standby");
      moveRecorded(recorded::standby);
      checkpoint();
      feedback("returned to standby");
      result->success = true;
      result->message = handle->get_goal()->side + " scan complete; returned to standby";
    } catch (const std::exception& error) {
      result->success = false;
      result->message = error.what();
    }
    result->completed_waypoints = completed_;
    if (rclcpp::ok()) {
      try {
        if (handle->is_canceling()) {
          result->success = false;
          result->message = "Canceled after current MoveIt operation";
          handle->canceled(result);
        } else if (result->success) {
          handle->succeed(result);
        } else {
          handle->abort(result);
        }
        RCLCPP_INFO(node_->get_logger(), "%s (%u waypoints)",
                    result->message.c_str(), completed_);
      } catch (const std::exception& error) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot return action result: %s", error.what());
      }
    }
    goal_.reset();
    busy_.store(false);
  }

  template <typename PoseList>
  void runRecorded(const PoseList& poses)
  {
    const size_t total = waypoint_limit_ == 0 ? poses.size() :
      std::min<size_t>(waypoint_limit_, poses.size());
    // Reject invalid input before moving; never clamp, wrap, or change limits.
    for (size_t i = 0; i < total; ++i) {
      const auto& pose = poses[i];
      state_.setVariablePositions(recorded::target(pose));
      state_.update();
      if (!state_.satisfiesBounds())
        throw std::runtime_error(std::string(pose.name) + ": joint limits exceeded");
    }
    markers_.markers.resize(3);
    for (size_t i = 0; i < total; ++i) {
      const auto& pose = poses[i];
      state_.setVariablePositions(recorded::target(pose));
      state_.update();
      label(pose.name, reference_.inverse() *
        state_.getGlobalLinkTransform("wrist3_link").translation(), 3 + i);
    }
    pub_->publish(markers_);
    for (size_t i = 0; i < total; ++i) {
      const auto& pose = poses[i];
      moveRecorded(pose);
      ++completed_;
      feedback(std::string("reached: ") + pose.name);
      feedback(std::string("holding 3 seconds: ") + pose.name);
      const auto deadline = std::chrono::steady_clock::now() + 3s;
      while (std::chrono::steady_clock::now() < deadline) {
        checkpoint();
        std::this_thread::sleep_for(50ms);
      }
      if (capture_client_) captureWaypoint(pose.name, total);
    }
  }

  void moveRecorded(const recorded::Pose& pose)
  {
    checkpoint();
    feedback(std::string("planning: ") + pose.name);
    group_.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan result;
    if (!group_.setJointValueTarget(recorded::target(pose)))
      throw std::runtime_error(std::string(pose.name) + ": invalid joint target");
    bool planned = false;
    for (int attempt = 1; attempt <= 3; ++attempt) {
      checkpoint();
      group_.setStartStateToCurrentState();
      if (group_.plan(result) == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(node_->get_logger(), "%s: plan attempt %d/3 failed", pose.name, attempt);
    }
    if (!planned)
      throw std::runtime_error(std::string(pose.name) + ": planning failed after 3 attempts");
    Joints degrees;
    const auto values = recorded::target(pose);
    for (const auto& name : NAMES) degrees.push_back(values.at(name) / RAD);
    execute(degrees, pose.name, result);
  }

  void captureWaypoint(const std::string& name, size_t total)
  {
    checkpoint();
    auto current = group_.getCurrentState(5.0);
    if (!current) throw std::runtime_error("No joint state for capture");
    auto request = std::make_shared<fairino_waypoint_test::srv::CaptureWaypoint::Request>();
    request->inspection_id = goal_->get_goal()->inspection_id;
    request->side = goal_->get_goal()->side;
    request->viewpoint = name;
    request->waypoint_index = completed_;
    request->total_waypoints = total;
    for (const auto& joint : NAMES)
      request->joints_deg.push_back(current->getVariablePosition(joint) / RAD);
    const auto& transform = current->getGlobalLinkTransform("wrist3_link");
    request->tool_pose.header.frame_id = group_.getRobotModel()->getModelFrame();
    request->tool_pose.header.stamp = node_->now();
    auto& pose = request->tool_pose.pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond q(transform.rotation());
    pose.orientation.x = q.x(); pose.orientation.y = q.y();
    pose.orientation.z = q.z(); pose.orientation.w = q.w();
    feedback("capturing: " + name);
    auto future = capture_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    while (future.wait_for(50ms) != std::future_status::ready) {
      checkpoint();
      if (std::chrono::steady_clock::now() >= deadline) {
        capture_client_->remove_pending_request(future);
        throw std::runtime_error("Capture timed out; scan stopped");
      }
    }
    auto response = future.get();
    if (!response->success)
      throw std::runtime_error("Capture failed: " + response->message);
    feedback("captured: " + name);
  }

  void showPlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = group_.getRobotModel()->getName();
    display.trajectory_start = plan.start_state_;
    display.trajectory.push_back(plan.trajectory_);
    trajectory_pub_->publish(display);
    Marker line;
    line.header.frame_id = group_.getRobotModel()->getModelFrame();
    line.ns = "planned_tool_paths";
    line.id = paths_.markers.size();
    line.type = Marker::LINE_STRIP;
    line.action = Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.006;
    line.color.r = line.id % 2 ? 1.0f : 0.1f;
    line.color.g = 0.8f;
    line.color.b = line.id % 2 ? 0.1f : 1.0f;
    line.color.a = 1.0f;
    moveit::core::RobotState sample(group_.getRobotModel());
    sample.setToDefaultValues();
    for (const auto& point : plan.trajectory_.joint_trajectory.points) {
      sample.setVariablePositions(plan.trajectory_.joint_trajectory.joint_names, point.positions);
      sample.update();
      const auto p = sample.getGlobalLinkTransform("wrist3_link").translation();
      geometry_msgs::msg::Point vertex;
      vertex.x = p.x(); vertex.y = p.y(); vertex.z = p.z();
      line.points.push_back(vertex);
    }
    paths_.markers.push_back(line);
    path_pub_->publish(paths_);
  }

  void execute(const Joints& joints, const std::string& name,
               const moveit::planning_interface::MoveGroupInterface::Plan& result)
  {
    checkpoint();
    showPlan(result);
    feedback("moving: " + name);
    std::ostringstream text;
    text << name << " [deg]:";
    for (double q : joints) text << ' ' << q;
    RCLCPP_INFO(node_->get_logger(), "%s", text.str().c_str());
    uint64_t before;
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      before = joint_sequence_;
    }
    if (group_.execute(result) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error(name + ": execution failed; test stopped");
    // A controller success alone is not proof that the physical arm arrived.
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    std::unique_lock<std::mutex> lock(joint_mutex_);
    while (true) {
      checkpoint();
      const auto now = std::chrono::steady_clock::now();
      bool reached = joint_sequence_ > before && now - joint_received_at_ < 500ms;
      for (size_t i = 0; i < NAMES.size(); ++i) {
        const auto found = measured_joints_.find(NAMES[i]);
        if (found == measured_joints_.end() || !std::isfinite(found->second) ||
            std::abs(found->second - joints.at(i) * RAD) > 0.02) reached = false;
      }
      if (reached) break;
      if (now >= deadline)
        throw std::runtime_error(name +
          ": measured /arm/joint_states did not reach target (0.02 rad tolerance); capture blocked");
      joint_updated_.wait_for(lock, 50ms);
    }
  }

  void label(const std::string& text, const Eigen::Vector3d& local, int id)
  {
    Marker marker;
    marker.header.frame_id = group_.getRobotModel()->getModelFrame();
    marker.ns = "waypoint_test";
    marker.id = id;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const Eigen::Vector3d p = reference_ * local;
    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.scale.z = 0.055;
    marker.color.r = 1.0;
    marker.color.g = 0.85;
    marker.color.b = 0.15;
    marker.color.a = 1.0;
    marker.text = text;
    // Avoid wide rendered spaces in RViz waypoint captions; keep source names
    // and action feedback unchanged. IDs 0..2 are the direction labels.
    if (id >= 3)
      std::replace(marker.text.begin(), marker.text.end(), ' ', '/');
    markers_.markers.push_back(marker);
  }

  rclcpp::Client<fairino_waypoint_test::srv::CaptureWaypoint>::SharedPtr capture_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::mutex joint_mutex_;
  std::condition_variable joint_updated_;
  std::map<std::string, double> measured_joints_;
  std::chrono::steady_clock::time_point joint_received_at_{};
  uint64_t joint_sequence_{0};
  std::atomic<bool> busy_{false};
  std::thread worker_;
  rclcpp_action::Server<Scan>::SharedPtr server_;
  std::shared_ptr<GoalHandle> goal_;
  uint32_t completed_{0};
  int64_t waypoint_limit_{0};
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface group_;
  moveit::core::RobotState state_;
  Eigen::Isometry3d reference_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  visualization_msgs::msg::MarkerArray markers_;
  visualization_msgs::msg::MarkerArray paths_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fairino_waypoint_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (!node->has_parameter("velocity_scale")) node->declare_parameter("velocity_scale", 0.08);
  if (!node->has_parameter("acceleration_scale")) node->declare_parameter("acceleration_scale", 0.03);
  if (!node->has_parameter("connection_timeout")) node->declare_parameter("connection_timeout", 30.0);
  if (!node->has_parameter("waypoint_limit")) node->declare_parameter("waypoint_limit", 0);
  for (const auto& parameter : std::map<std::string, double>{
      {"wall_distance", 0.5}, {"wall_thickness", 0.02}, {"wall_width", 1.0}, {"wall_height", 1.0}})
    if (!node->has_parameter(parameter.first)) node->declare_parameter(parameter.first, parameter.second);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::exception_ptr spin_error;
  std::thread spinner([&executor, &spin_error] {
    try {
      executor.spin();
    } catch (...) {
      spin_error = std::current_exception();
      if (rclcpp::ok()) rclcpp::shutdown();
    }
  });
  int status = 0;
  try {
    if (waitForMoveIt(node)) {
      Test test(node);
      test.startServer();
      // Keep transient-local markers available until the user closes the launch.
      while (rclcpp::ok()) std::this_thread::sleep_for(100ms);
    }
  } catch (const std::exception& error) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "%s", error.what());
      status = 1;
    }
  }
  executor.cancel();
  spinner.join();
  if (spin_error) {
    try { std::rethrow_exception(spin_error); }
    catch (const std::exception& error) {
      RCLCPP_ERROR(node->get_logger(), "Executor stopped: %s", error.what());
    }
    status = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
