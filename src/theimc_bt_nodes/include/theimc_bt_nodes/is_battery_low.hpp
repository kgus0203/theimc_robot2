#ifndef THEIMC_BT_NODES__IS_BATTERY_LOW_HPP_
#define THEIMC_BT_NODES__IS_BATTERY_LOW_HPP_

#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace theimc_bt_nodes
{

class IsBatteryLow : public BT::ConditionNode
{
public:
  IsBatteryLow(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;

  std::mutex mutex_;
  double current_battery_{100.0};
  bool is_battery_low_{false};
  bool has_received_msg_{false};
  bool has_received_battery_state_{false};
};

}  // namespace theimc_bt_nodes

#endif  // THEIMC_BT_NODES__IS_BATTERY_LOW_HPP_