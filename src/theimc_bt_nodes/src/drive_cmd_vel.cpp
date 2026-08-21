#include "theimc_bt_nodes/drive_cmd_vel.hpp"

#include <cmath>

namespace theimc_bt_nodes
{

DriveCmdVel::DriveCmdVel(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(xml_tag_name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_bt", rclcpp::QoS(10));
}

BT::PortsList DriveCmdVel::providedPorts()
{
  return {
    BT::InputPort<double>("linear_x", 0.0, "Linear x velocity in m/s"),
    BT::InputPort<double>("angular_z", 0.0, "Angular z velocity in rad/s"),
    BT::InputPort<double>("seconds", "Duration to publish velocity"),
  };
}

BT::NodeStatus DriveCmdVel::onStart()
{
  if (!getInput("linear_x", linear_x_) || !getInput("angular_z", angular_z_) ||
    !getInput("seconds", seconds_) || !std::isfinite(linear_x_) ||
    !std::isfinite(angular_z_) || !std::isfinite(seconds_) || seconds_ <= 0.0)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "DriveCmdVel requires finite velocities and seconds > 0");
    stop();
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node_->now();
  active_ = true;
  publishVelocity(linear_x_, angular_z_);
  RCLCPP_INFO(
    node_->get_logger(),
    "DriveCmdVel started: linear_x=%.3f, angular_z=%.3f, seconds=%.3f",
    linear_x_, angular_z_, seconds_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveCmdVel::onRunning()
{
  if ((node_->now() - start_time_).seconds() >= seconds_) {
    stop();
    RCLCPP_INFO(node_->get_logger(), "DriveCmdVel completed and published zero velocity");
    return BT::NodeStatus::SUCCESS;
  }

  // Keep publishing while RUNNING so downstream velocity timeouts do not stop early.
  publishVelocity(linear_x_, angular_z_);
  return BT::NodeStatus::RUNNING;
}

void DriveCmdVel::onHalted()
{
  if (active_) {
    stop();
    RCLCPP_WARN(node_->get_logger(), "DriveCmdVel halted and published zero velocity");
  }
}

void DriveCmdVel::publishVelocity(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  publisher_->publish(msg);
}

void DriveCmdVel::stop()
{
  publishVelocity(0.0, 0.0);
  active_ = false;
}

}  // namespace theimc_bt_nodes
