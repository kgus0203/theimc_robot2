#include "theimc_bt_nodes/return_home_requested.hpp"

#include <string>

namespace theimc_bt_nodes
{

ReturnHomeRequested::ReturnHomeRequested(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(xml_tag_name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  return_home_subscription_ =
    node_->create_subscription<std_msgs::msg::Bool>(
    "/return_home",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg)
    {
      if (!msg->data) {
        return;
      }

      std::lock_guard<std::mutex> lock(mutex_);
      return_home_requested_ = true;

      RCLCPP_WARN(node_->get_logger(), "Return-home request received");
    },
    options);
}

BT::PortsList ReturnHomeRequested::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "current_mode",
      "DIRECT",
      "Current mission return mode"),

    BT::OutputPort<std::string>(
      "return_mode",
      "DIRECT, REVERSE_HOME, or EXIT_RAIL_HOME")
  };
}

BT::NodeStatus ReturnHomeRequested::tick()
{
  callback_group_executor_.spin_some();

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!return_home_requested_) {
      return BT::NodeStatus::FAILURE;
    }

    return_home_requested_ = false;
  }

  std::string current_mode = "DIRECT";

  if (!getInput("current_mode", current_mode)) {
    current_mode = "DIRECT";
    RCLCPP_WARN(
      node_->get_logger(),
      "ReturnHomeRequested: current_mode unavailable; using DIRECT");
  }

  std::string return_mode = "DIRECT";

  if (
    current_mode == "REVERSE_HOME" ||
    current_mode == "EXIT_RAIL_HOME")
  {
    return_mode = current_mode;
  }

  if (!setOutput("return_mode", return_mode)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ReturnHomeRequested: failed to write return_mode output");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "Starting return-home flow: mode=%s, current_mode=%s",
    return_mode.c_str(),
    current_mode.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace theimc_bt_nodes