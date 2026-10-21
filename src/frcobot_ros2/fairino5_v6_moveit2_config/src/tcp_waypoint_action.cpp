#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
std::vector<geometry_msgs::msg::Pose> buildWaypoints(
    const std::vector<double>& points, const geometry_msgs::msg::Quaternion& orientation)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(points.size() / 3);

  for (std::size_t i = 0; i + 2 < points.size(); i += 3) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = points[i];
    pose.position.y = points[i + 1];
    pose.position.z = points[i + 2];
    pose.orientation = orientation;
    waypoints.push_back(pose);
  }

  return waypoints;
}

bool validPoints(const std::vector<double>& points)
{
  if (points.size() != 12) {
    return false;
  }

  for (const double value : points) {
    if (!std::isfinite(value)) {
      return false;
    }
  }

  return true;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("tcp_waypoint_action");

  const auto group_name = node->declare_parameter<std::string>("group_name", "fairino5_v6_group");
  const auto tcp_link = node->declare_parameter<std::string>("tcp_link", "wrist3_link");
  const auto reference_frame = node->declare_parameter<std::string>("reference_frame", "base_link");
  const auto command_topic = node->declare_parameter<std::string>("command_topic", "/arm_command");
  const auto start_command = node->declare_parameter<std::string>("start_command", "START");
  const auto velocity_scaling = node->declare_parameter<double>("velocity_scaling", 0.2);
  const auto acceleration_scaling = node->declare_parameter<double>("acceleration_scaling", 0.2);
  const auto eef_step = node->declare_parameter<double>("eef_step", 0.01);
  const auto jump_threshold = node->declare_parameter<double>("jump_threshold", 0.0);
  const auto min_path_fraction = node->declare_parameter<double>("min_path_fraction", 0.95);
  const auto use_current_orientation = node->declare_parameter<bool>("use_current_orientation", true);
  const auto points = node->declare_parameter<std::vector<double>>(
      "rectangle_points",
      {
          0.35, -0.10, 0.35,
          0.35, 0.10, 0.35,
          0.35, 0.10, 0.20,
          0.35, -0.10, 0.20,
      });
  const auto orientation_values = node->declare_parameter<std::vector<double>>(
      "target_orientation_xyzw", {0.0, 0.0, 0.0, 1.0});

  if (!validPoints(points)) {
    RCLCPP_FATAL(node->get_logger(),
                 "Parameter 'rectangle_points' must contain exactly 12 finite values: "
                 "[x1,y1,z1,...,x4,y4,z4] in meters.");
    rclcpp::shutdown();
    return 1;
  }

  if ((!use_current_orientation && orientation_values.size() != 4) ||
      (use_current_orientation && !orientation_values.empty() && orientation_values.size() != 4)) {
    RCLCPP_FATAL(node->get_logger(),
                 "Parameter 'target_orientation_xyzw' must be empty or contain exactly 4 values: "
                 "[x,y,z,w].");
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
  move_group.setEndEffectorLink(tcp_link);
  move_group.setPoseReferenceFrame(reference_frame);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling);

  std::atomic_bool busy{false};

  auto execute_sequence = [&]() {
    if (busy.exchange(true)) {
      RCLCPP_WARN(node->get_logger(), "Command ignored because a waypoint sequence is already running.");
      return;
    }

    try {
      RCLCPP_INFO(node->get_logger(), "START received. Planning 4 fixed rectangle waypoints.");

      const auto current_pose = move_group.getCurrentPose(tcp_link).pose;
      geometry_msgs::msg::Quaternion target_orientation = current_pose.orientation;
      if (!use_current_orientation) {
        target_orientation.x = orientation_values[0];
        target_orientation.y = orientation_values[1];
        target_orientation.z = orientation_values[2];
        target_orientation.w = orientation_values[3];
      }

      const auto waypoints = buildWaypoints(points, target_orientation);
      if (waypoints.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No rectangle waypoints configured.");
        busy = false;
        return;
      }

      move_group.setPoseTarget(waypoints.front(), tcp_link);
      moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
      const auto approach_result = move_group.plan(approach_plan);
      move_group.clearPoseTargets();

      if (approach_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan approach to the first rectangle point.");
        busy = false;
        return;
      }

      const auto approach_execute_result = move_group.execute(approach_plan);
      if (approach_execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to execute approach to the first rectangle point.");
        busy = false;
        return;
      }

      std::vector<geometry_msgs::msg::Pose> rectangle_path;
      rectangle_path.reserve(waypoints.size());
      rectangle_path.insert(rectangle_path.end(), waypoints.begin() + 1, waypoints.end());
      rectangle_path.push_back(waypoints.front());

      moveit_msgs::msg::RobotTrajectory trajectory;
      const double fraction =
          move_group.computeCartesianPath(rectangle_path, eef_step, jump_threshold, trajectory, true);

      if (fraction < min_path_fraction) {
        RCLCPP_ERROR(node->get_logger(), "Cartesian path planning failed: fraction %.3f < %.3f",
                     fraction, min_path_fraction);
        busy = false;
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      const auto result = move_group.execute(plan);

      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "TCP waypoint sequence completed.");
      } else {
        RCLCPP_ERROR(node->get_logger(), "TCP waypoint sequence execution failed.");
      }
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(node->get_logger(), "TCP waypoint sequence failed: %s", ex.what());
    }

    busy = false;
  };

  auto subscription = node->create_subscription<std_msgs::msg::String>(
      command_topic, rclcpp::QoS(10),
      [&](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data != start_command) {
          RCLCPP_DEBUG(node->get_logger(), "Ignoring command '%s'.", msg->data.c_str());
          return;
        }
        std::thread(execute_sequence).detach();
      });

  RCLCPP_INFO(node->get_logger(), "Listening for '%s' on %s.", start_command.c_str(),
              command_topic.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
