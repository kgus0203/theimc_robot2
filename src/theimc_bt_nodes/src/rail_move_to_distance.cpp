#include "theimc_bt_nodes/rail_move_to_distance.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace theimc_bt_nodes
{

RailMoveToDistance::RailMoveToDistance(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(xml_tag_name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error(
            "RailMoveToDistance: ROS node was not found in BT blackboard");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/rail/odom",
    rclcpp::QoS(10),
    std::bind(
      &RailMoveToDistance::odomCallback,
      this,
      std::placeholders::_1),
    options);

  obstacle_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/rail_obstacle",
    rclcpp::QoS(10),
    std::bind(
      &RailMoveToDistance::obstacleCallback,
      this,
      std::placeholders::_1),
    options);

  command_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/rail_command",
    rclcpp::QoS(10));

  // Test/work status publishers.
  // They do not control the robot; they only expose BT progress.
  target_goal_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "/rail_target_goal_m",
    rclcpp::QoS(10));

  target_reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/rail_target_reached",
    rclcpp::QoS(10));

  target_reached_m_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "/rail_target_reached_m",
    rclcpp::QoS(10));
}

BT::PortsList RailMoveToDistance::providedPorts()
{
  return {
    BT::InputPort<double>(
      "target_m",
      "Absolute target distance on /rail/odom"),

    BT::InputPort<double>(
      "tolerance_m",
      0.05,
      "Target distance tolerance"),

    BT::InputPort<double>(
      "timeout_sec",
      0.0,
      "Move timeout excluding obstacle wait time. 0 means no timeout"),

    BT::InputPort<double>(
      "obstacle_wait_timeout_sec",
      0.0,
      "Maximum continuous obstacle wait. 0 means wait indefinitely"),

    BT::InputPort<double>(
      "odom_stale_sec",
      0.5,
      "Maximum accepted /rail/odom age"),

    BT::OutputPort<double>(
      "reached_m",
      "Actual distance when target is reached")
  };
}

void RailMoveToDistance::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double distance =
    static_cast<double>(msg->pose.pose.position.x);

  if (!std::isfinite(distance)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  current_distance_m_ = distance;
  odom_time_ = node_->now();
  odom_received_ = true;
}

void RailMoveToDistance::obstacleCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_ = msg->data;
}

bool RailMoveToDistance::getFreshDistance(double & distance_m)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!odom_received_) {
    return false;
  }

  if ((node_->now() - odom_time_).seconds() > odom_stale_sec_) {
    return false;
  }

  distance_m = current_distance_m_;
  return true;
}

void RailMoveToDistance::publishCommand(
  const std::string & command)
{
  // Avoid flooding identical string commands every BT tick.
  if (command == last_command_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = command;
  command_pub_->publish(msg);

  last_command_ = command;

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailMoveToDistance] command => %s",
    command.c_str());
}

void RailMoveToDistance::stop()
{
  publishCommand("STOP");
}

void RailMoveToDistance::publishTargetStarted()
{
  std_msgs::msg::Float32 goal_msg;
  goal_msg.data = static_cast<float>(target_m_);
  target_goal_pub_->publish(goal_msg);

  // Reset the success signal for this new target.
  std_msgs::msg::Bool reached_msg;
  reached_msg.data = false;
  target_reached_pub_->publish(reached_msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailMoveToDistance] TARGET START target=%.3f m",
    target_m_);
}

void RailMoveToDistance::publishTargetReached(double reached_m)
{
  std_msgs::msg::Float32 reached_m_msg;
  reached_m_msg.data = static_cast<float>(reached_m);
  target_reached_m_pub_->publish(reached_m_msg);

  std_msgs::msg::Bool reached_msg;
  reached_msg.data = true;
  target_reached_pub_->publish(reached_msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailMoveToDistance] TARGET SUCCESS target=%.3f m reached=%.3f m",
    target_m_,
    reached_m);
}

BT::NodeStatus RailMoveToDistance::onStart()
{
  if (!getInput("target_m", target_m_)) {
    throw BT::RuntimeError(
            "RailMoveToDistance: missing required input [target_m]");
  }

  getInput("tolerance_m", tolerance_m_);
  getInput("timeout_sec", timeout_sec_);
  getInput("obstacle_wait_timeout_sec", obstacle_wait_timeout_sec_);
  getInput("odom_stale_sec", odom_stale_sec_);

  if (
    target_m_ < 0.0 ||
    tolerance_m_ <= 0.0 ||
    timeout_sec_ < 0.0 ||
    obstacle_wait_timeout_sec_ < 0.0 ||
    odom_stale_sec_ <= 0.0)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[RailMoveToDistance] Invalid input parameters");

    stop();
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node_->now();
  obstacle_waiting_ = false;
  accumulated_obstacle_wait_sec_ = 0.0;
  last_command_.clear();

  publishTargetStarted();

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailMoveToDistance] START target=%.3f m tolerance=%.3f m",
    target_m_,
    tolerance_m_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RailMoveToDistance::onRunning()
{
  callback_group_executor_.spin_some();

  const rclcpp::Time now = node_->now();

  bool obstacle_now = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_now = obstacle_;
  }

  // --------------------------------------------------------------
  // Obstacle handling
  // --------------------------------------------------------------
  // While an obstacle is present:
  //   1) STOP the rail motor,
  //   2) keep this BT node RUNNING,
  //   3) do NOT consume timeout_sec,
  //   4) resume the same target automatically when the obstacle clears.
  if (obstacle_now) {
    if (!obstacle_waiting_) {
      obstacle_waiting_ = true;
      obstacle_wait_start_ = now;

      stop();

      RCLCPP_WARN(
        node_->get_logger(),
        "[RailMoveToDistance] Obstacle detected -> STOP and WAIT "
        "(movement timeout paused)");
    } else {
      stop();
    }

    if (
      obstacle_wait_timeout_sec_ > 0.0 &&
      (now - obstacle_wait_start_).seconds() >= obstacle_wait_timeout_sec_)
    {
      stop();

      RCLCPP_ERROR(
        node_->get_logger(),
        "[RailMoveToDistance] Obstacle wait timeout after %.1f sec",
        obstacle_wait_timeout_sec_);

      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  // Transition WAIT_OBSTACLE -> MOVING.
  if (obstacle_waiting_) {
    const double waited_sec =
      std::max(0.0, (now - obstacle_wait_start_).seconds());

    accumulated_obstacle_wait_sec_ += waited_sec;
    obstacle_waiting_ = false;

    RCLCPP_INFO(
      node_->get_logger(),
      "[RailMoveToDistance] Obstacle cleared after %.2f sec "
      "-> resume target %.3f m",
      waited_sec,
      target_m_);
  }

  // timeout_sec counts only non-obstacle time.
  const double elapsed_sec =
    std::max(0.0, (now - start_time_).seconds());

  const double effective_move_elapsed_sec =
    std::max(0.0, elapsed_sec - accumulated_obstacle_wait_sec_);

  if (
    timeout_sec_ > 0.0 &&
    effective_move_elapsed_sec >= timeout_sec_)
  {
    stop();

    RCLCPP_ERROR(
      node_->get_logger(),
      "[RailMoveToDistance] Move timeout: active=%.2f sec "
      "(obstacle wait excluded=%.2f sec)",
      effective_move_elapsed_sec,
      accumulated_obstacle_wait_sec_);

    return BT::NodeStatus::FAILURE;
  }

  double distance_m = 0.0;

  if (!getFreshDistance(distance_m)) {
    stop();

    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "[RailMoveToDistance] /rail/odom stale or missing - waiting");

    return BT::NodeStatus::RUNNING;
  }

  const double error = target_m_ - distance_m;

  if (std::abs(error) <= tolerance_m_) {
    // Stop first, then report success.
    stop();
    setOutput("reached_m", distance_m);
    publishTargetReached(distance_m);

    return BT::NodeStatus::SUCCESS;
  }

  // last_command_ is STOP after obstacle waiting, so the first tick after
  // obstacle clear automatically republishes FORWARD/BACK.
  if (error > 0.0) {
    publishCommand("FORWARD");
  } else {
    publishCommand("BACK");
  }

  return BT::NodeStatus::RUNNING;
}

void RailMoveToDistance::onHalted()
{
  stop();

  RCLCPP_WARN(
    node_->get_logger(),
    "[RailMoveToDistance] Halted - STOP sent");
}

}  // namespace theimc_bt_nodes