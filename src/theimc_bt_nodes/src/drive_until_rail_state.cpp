#include "theimc_bt_nodes/drive_until_rail_state.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace theimc_bt_nodes
{
namespace
{

std::string upperCopy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return value;
}

}  // namespace

DriveUntilRailState::DriveUntilRailState(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(xml_tag_name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Same chassis command path used by DriveCmdVel / BringUp.
  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10));

  rail_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/rail_state",
    rclcpp::QoS(20),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      rail_state_ = upperCopy(msg->data);
    });
}

BT::PortsList DriveUntilRailState::providedPorts()
{
  return {
    BT::InputPort<double>("linear_x", 0.0, "Chassis linear velocity (m/s)"),
    BT::InputPort<double>("angular_z", 0.0, "Chassis angular velocity (rad/s)"),
    BT::InputPort<std::string>("target_state", "ON_RAIL", "Rail state that stops the chassis"),
    BT::InputPort<double>("timeout_sec", 0.0, "0 disables timeout"),
  };
}

BT::NodeStatus DriveUntilRailState::onStart()
{
  if (!getInput("linear_x", linear_x_) ||
    !getInput("angular_z", angular_z_) ||
    !getInput("target_state", target_state_) ||
    !getInput("timeout_sec", timeout_sec_) ||
    !std::isfinite(linear_x_) || !std::isfinite(angular_z_) ||
    !std::isfinite(timeout_sec_) || timeout_sec_ < 0.0)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[DriveUntilRailState] invalid input port(s)");
    stop();
    return BT::NodeStatus::FAILURE;
  }

  target_state_ = upperCopy(target_state_);
  start_time_ = node_->now();
  active_ = true;

  const auto current = currentRailState();
  if (current == target_state_) {
    stop();
    RCLCPP_INFO(
      node_->get_logger(),
      "[DriveUntilRailState] target already reached: %s",
      target_state_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // Important: publish immediately when the EnterRail Parallel starts.
  publishVelocity(linear_x_, angular_z_);
  RCLCPP_INFO(
    node_->get_logger(),
    "[DriveUntilRailState] START vx=%.3f wz=%.3f target=%s timeout=%.3f",
    linear_x_, angular_z_, target_state_.c_str(), timeout_sec_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveUntilRailState::onRunning()
{
  const auto current = currentRailState();

  if (current == target_state_) {
    stop();
    RCLCPP_INFO(
      node_->get_logger(),
      "[DriveUntilRailState] target reached: %s -> STOP",
      target_state_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_sec_ > 0.0 &&
    (node_->now() - start_time_).seconds() >= timeout_sec_)
  {
    stop();
    RCLCPP_ERROR(
      node_->get_logger(),
      "[DriveUntilRailState] timeout waiting for %s (current=%s)",
      target_state_.c_str(), current.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Re-publish every BT tick. A single Twist can otherwise be lost or be
  // superseded by downstream velocity timeouts/mux behavior.
  publishVelocity(linear_x_, angular_z_);
  return BT::NodeStatus::RUNNING;
}

void DriveUntilRailState::onHalted()
{
  if (active_) {
    stop();
    RCLCPP_WARN(
      node_->get_logger(),
      "[DriveUntilRailState] HALTED -> STOP");
  }
}

void DriveUntilRailState::publishVelocity(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  publisher_->publish(msg);
}

void DriveUntilRailState::stop()
{
  if (publisher_) {
    publishVelocity(0.0, 0.0);
  }
  active_ = false;
}

std::string DriveUntilRailState::currentRailState()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return rail_state_;
}

}  // namespace theimc_bt_nodes