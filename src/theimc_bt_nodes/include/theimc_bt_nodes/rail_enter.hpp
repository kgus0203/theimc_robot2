#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace theimc_bt_nodes
{

class RailEnter : public BT::StatefulActionNode
{
public:
  RailEnter(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    WAIT_ENTRY_LOW,   // 레일 접근 완료 후 100~120 mm 진입 신호 대기
    SETTLING,         // 진입 감지 직후 잠깐 정지
    ENTERING          // 레일 모터를 전진시키며 다시 높은 TOF 대기
  };

  void tofCallback(const std_msgs::msg::Float32::SharedPtr msg);

  bool getFreshTof(double & distance_mm);
  bool timedOut() const;

  void publishBaseStop();
  void publishRailVelocity(double velocity);
  void publishRailState(const std::string & state);
  void stopOutputs();

  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_rail_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rail_state_pub_;

  std::mutex tof_mutex_;
  bool tof_received_{false};
  double latest_tof_mm_{0.0};
  rclcpp::Time latest_tof_time_;

  Phase phase_{Phase::WAIT_ENTRY_LOW};

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;

  // Parameters / BT ports
  double enter_tof_mm_{120.0};
  double on_rail_tof_mm_{160.0};
  double settle_sec_{2.0};
  double rail_velocity_{0.3};
  double timeout_sec_{0.0};
  double tof_stale_sec_{0.5};

  int enter_confirm_samples_{2};
  int on_rail_confirm_samples_{3};

  int enter_low_count_{0};
  int on_rail_high_count_{0};
};

}  // namespace theimc_bt_nodes