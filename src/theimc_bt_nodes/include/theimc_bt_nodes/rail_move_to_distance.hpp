#ifndef THEIMC_BT_NODES__RAIL_MOVE_TO_DISTANCE_HPP_
#define THEIMC_BT_NODES__RAIL_MOVE_TO_DISTANCE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace theimc_bt_nodes
{

class RailMoveToDistance : public BT::StatefulActionNode
{
public:
  RailMoveToDistance(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg);

  bool getFreshDistance(double & distance_m);
  void publishCommand(const std::string & command);
  void stop();

  // Test/work status topics:
  //   /rail_target_goal_m      Float32 : current target position
  //   /rail_target_reached     Bool    : false while moving, true on success
  //   /rail_target_reached_m   Float32 : actual position at success
  void publishTargetStarted();
  void publishTargetReached(double reached_m);

  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_reached_m_pub_;

  std::mutex mutex_;
  double current_distance_m_{0.0};
  rclcpp::Time odom_time_;
  bool odom_received_{false};
  bool obstacle_{false};

  double target_m_{0.0};
  double tolerance_m_{0.05};
  double timeout_sec_{0.0};
  double odom_stale_sec_{0.5};
  rclcpp::Time start_time_;

  std::string last_command_;
};

}  // namespace theimc_bt_nodes

#endif  // THEIMC_BT_NODES__RAIL_MOVE_TO_DISTANCE_HPP_