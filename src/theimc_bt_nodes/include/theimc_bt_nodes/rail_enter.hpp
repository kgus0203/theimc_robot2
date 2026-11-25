#ifndef THEIMC_BT_NODES__RAIL_ENTER_HPP_
#define THEIMC_BT_NODES__RAIL_ENTER_HPP_

#include <array>
#include <chrono>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace theimc_bt_nodes
{

class RailEnter : public BT::StatefulActionNode
{
public:
  RailEnter(const std::string & xml_tag_name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { WAIT_ENTRY_LOW, WAIT_ON_RAIL_HIGH, WAIT_POWER_READY };
  void tofCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void axisDiagnosticsCallback(const std_msgs::msg::String::SharedPtr msg);
  void agvStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  bool getFreshTof(double & distance_mm);
  bool timedOut() const;
  void publishRailState(const std::string & state);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr axis_diag_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr agv_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rail_state_pub_;
  std::mutex tof_mutex_;
  bool tof_received_{false};
  double latest_tof_mm_{0.0};
  rclcpp::Time latest_tof_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  Phase phase_{Phase::WAIT_ENTRY_LOW};
  double enter_tof_mm_{120.0};
  double on_rail_tof_mm_{270.0};
  double timeout_sec_{30.0};
  double tof_stale_sec_{0.5};
  int enter_confirm_samples_{2};
  int on_rail_confirm_samples_{3};
  int enter_low_count_{0};
  int on_rail_high_count_{0};
  double power_ready_timeout_sec_{12.0};
  double power_feedback_stale_sec_{1.0};
  std::array<bool, 3> axis_ready_{{false, false, false}};
  bool axis_diag_received_{false};
  bool agv_status_received_{false};
  bool agv_status_ready_{false};
  bool power_timeout_reported_{false};
  std::chrono::steady_clock::time_point axis_diag_received_at_{};
  std::chrono::steady_clock::time_point agv_status_received_at_{};
  std::chrono::steady_clock::time_point power_wait_started_{};
};

}  // namespace theimc_bt_nodes
#endif  // THEIMC_BT_NODES__RAIL_ENTER_HPP_