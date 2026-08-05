#ifndef THEIMC_BT_NODES__IS_OBSTACLE_DETECTED_HPP_
#define THEIMC_BT_NODES__IS_OBSTACLE_DETECTED_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace theimc_bt_nodes
{

class IsObstacleDetected : public BT::ConditionNode
{
public:
  IsObstacleDetected(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  // BT에서 이 노드가 실행될 때마다 호출되는 함수
  BT::NodeStatus tick() override;

private:
  // 라이다 데이터를 지속적으로 받아올 콜백 함수
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex mutex_;
};

}  // namespace theimc_bt_nodes

#endif  // THEIMC_BT_NODES__IS_OBSTACLE_DETECTED_HPP_