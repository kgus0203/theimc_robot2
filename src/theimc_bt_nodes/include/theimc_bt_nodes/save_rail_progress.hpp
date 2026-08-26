#pragma once

#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace theimc_bt_nodes
{

class SaveRailProgress : public BT::SyncActionNode
{
public:
  SaveRailProgress(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg);

  bool getFreshDistance(
    double max_age_sec,
    double & distance_m);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::mutex mutex_;
  bool odom_received_{false};
  double current_distance_m_{0.0};
  rclcpp::Time odom_time_;
};

}  // namespace theimc_bt_nodes
