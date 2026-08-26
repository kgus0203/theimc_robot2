#ifndef THEIMC_BT_NODES__DRIVE_UNTIL_RAIL_STATE_HPP_
#define THEIMC_BT_NODES__DRIVE_UNTIL_RAIL_STATE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace theimc_bt_nodes
{

class DriveUntilRailState : public BT::StatefulActionNode
{
public:
  DriveUntilRailState(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publishVelocity(double linear_x, double angular_z);
  void stop();
  std::string currentRailState();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rail_state_sub_;

  std::mutex state_mutex_;
  std::string rail_state_{"UNKNOWN"};

  double linear_x_{0.0};
  double angular_z_{0.0};
  double timeout_sec_{0.0};
  std::string target_state_{"ON_RAIL"};
  rclcpp::Time start_time_;
  bool active_{false};
};

}  // namespace theimc_bt_nodes

#endif  // THEIMC_BT_NODES__DRIVE_UNTIL_RAIL_STATE_HPP_