#pragma once

#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace theimc_bt_nodes
{

class RailExit : public BT::StatefulActionNode
{
public:
  RailExit(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    WAIT_EXIT_LOW,
    EXITING
  };

  void tofCallback(const std_msgs::msg::Float32::SharedPtr msg);

  bool getFreshTof(double & distance_mm);
  bool timedOut() const;

  void publishRailState(const std::string & state);

  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rail_state_pub_;

  std::mutex tof_mutex_;
  bool tof_received_{false};
  double latest_tof_mm_{0.0};
  rclcpp::Time latest_tof_time_;

  Phase phase_{Phase::WAIT_EXIT_LOW};

  rclcpp::Time start_time_;

  double exit_tof_mm_{120.0};
  double out_rail_tof_mm_{160.0};
  double timeout_sec_{0.0};
  double tof_stale_sec_{0.5};

  int exit_confirm_samples_{2};
  int out_rail_confirm_samples_{3};

  int exit_low_count_{0};
  int out_rail_high_count_{0};
};

}  // namespace theimc_bt_nodes
