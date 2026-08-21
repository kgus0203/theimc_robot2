#include "theimc_bt_nodes/is_rail_obstacle.hpp"

#include <functional>
#include <stdexcept>

namespace theimc_bt_nodes
{

IsRailObstacle::IsRailObstacle(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(xml_tag_name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("Missing Node in blackboard");
  }

  obstacle_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/rail_obstacle",
    rclcpp::QoS(10),
    std::bind(
      &IsRailObstacle::obstacleCallback,
      this,
      std::placeholders::_1));
}

BT::PortsList IsRailObstacle::providedPorts()
{
  return {};
}

void IsRailObstacle::obstacleCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_ = msg->data;
  received_ = true;
}

BT::NodeStatus IsRailObstacle::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 아직 /rail_obstacle 상태를 한 번도 받지 못했다면
  // 장애물 분기로 진입하지 않습니다.
  if (!received_) {
    return BT::NodeStatus::FAILURE;
  }

  return obstacle_
    ? BT::NodeStatus::SUCCESS
    : BT::NodeStatus::FAILURE;
}

}  // namespace theimc_bt_nodes
