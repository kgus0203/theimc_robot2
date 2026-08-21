#pragma once

#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace theimc_bt_nodes
{

class WaitForRailClear : public BT::StatefulActionNode
{
public:
  WaitForRailClear(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void obstacleCallback(
    const std_msgs::msg::Bool::SharedPtr msg);

  BT::NodeStatus checkState();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;

  std::mutex mutex_;
  bool obstacle_{false};
  bool received_{false};
};

}  // namespace theimc_bt_nodes
