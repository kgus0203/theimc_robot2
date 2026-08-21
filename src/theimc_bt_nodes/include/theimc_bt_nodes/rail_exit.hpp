#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace theimc_bt_nodes
{

class RailExit : public BT::StatefulActionNode
{
public:
  RailExit(const std::string & xml_tag_name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void tofCallback(const std_msgs::msg::Float32::SharedPtr msg);
  BT::NodeStatus checkExit();
  bool timedOut() const;
  void publishRailState(const std::string & state);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rail_state_pub_;

  std::mutex tof_mutex_;
  double latest_tof_mm_{0.0};
  bool tof_received_{false};
  uint64_t tof_sequence_{0};
  rclcpp::Time latest_tof_time_;

  uint64_t start_sequence_{0};
  rclcpp::Time start_time_;

  double out_tof_mm_{140.0};
  double timeout_sec_{0.0};
  double tof_stale_sec_{0.5};
};

}  // namespace theimc_bt_nodes
