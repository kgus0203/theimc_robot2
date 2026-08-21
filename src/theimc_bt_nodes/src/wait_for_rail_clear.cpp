#include "theimc_bt_nodes/wait_for_rail_clear.hpp"

#include <functional>
#include <stdexcept>

namespace theimc_bt_nodes
{

WaitForRailClear::WaitForRailClear(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(xml_tag_name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("Missing Node in blackboard");
  }

  obstacle_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/rail_obstacle",
    rclcpp::QoS(10),
    std::bind(
      &WaitForRailClear::obstacleCallback,
      this,
      std::placeholders::_1));
}

BT::PortsList WaitForRailClear::providedPorts()
{
  return {};
}

void WaitForRailClear::obstacleCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_ = msg->data;
  received_ = true;
}

BT::NodeStatus WaitForRailClear::onStart()
{
  return checkState();
}

BT::NodeStatus WaitForRailClear::onRunning()
{
  return checkState();
}

void WaitForRailClear::onHalted()
{
}

BT::NodeStatus WaitForRailClear::checkState()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 아직 상태를 못 받았으면 안전하게 계속 대기합니다.
  if (!received_) {
    return BT::NodeStatus::RUNNING;
  }

  // 장애물이 남아 있으면 계속 대기합니다.
  if (obstacle_) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForRailClear] Rail obstacle cleared");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace theimc_bt_nodes
