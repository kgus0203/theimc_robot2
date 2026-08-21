#include "theimc_bt_nodes/rail_exit.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

namespace theimc_bt_nodes
{

RailExit::RailExit(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(xml_tag_name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("RailExit: missing ROS node in blackboard");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  tof_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/tof_distance", rclcpp::QoS(10),
    std::bind(&RailExit::tofCallback, this, std::placeholders::_1), options);

  rail_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/rail_state", rclcpp::QoS(10));
}

BT::PortsList RailExit::providedPorts()
{
  return {
    BT::InputPort<double>("out_tof_mm", 140.0, "TOF threshold for OUT_RAIL"),
    BT::InputPort<double>("timeout_sec", 0.0, "Timeout; 0 means wait indefinitely"),
    BT::InputPort<double>("tof_stale_sec", 0.5, "Maximum valid TOF age"),
  };
}

void RailExit::tofCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  const double value = static_cast<double>(msg->data);
  if (!std::isfinite(value) || value < 0.0) {
    return;
  }
  std::lock_guard<std::mutex> lock(tof_mutex_);
  latest_tof_mm_ = value;
  latest_tof_time_ = node_->now();
  tof_received_ = true;
  ++tof_sequence_;
}

BT::NodeStatus RailExit::onStart()
{
  getInput("out_tof_mm", out_tof_mm_);
  getInput("timeout_sec", timeout_sec_);
  getInput("tof_stale_sec", tof_stale_sec_);

  if (!std::isfinite(out_tof_mm_) || out_tof_mm_ <= 0.0 ||
      !std::isfinite(timeout_sec_) || timeout_sec_ < 0.0 ||
      !std::isfinite(tof_stale_sec_) || tof_stale_sec_ <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "RailExit: invalid input ports");
    return BT::NodeStatus::FAILURE;
  }

  callback_group_executor_.spin_some();
  {
    std::lock_guard<std::mutex> lock(tof_mutex_);
    start_sequence_ = tof_sequence_;
  }
  start_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailExit] Waiting for NEW TOF >= %.1f mm",
    out_tof_mm_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RailExit::onRunning()
{
  return checkExit();
}

void RailExit::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[RailExit] Halted");
}

BT::NodeStatus RailExit::checkExit()
{
  callback_group_executor_.spin_some();

  if (timedOut()) {
    RCLCPP_ERROR(node_->get_logger(), "[RailExit] Timeout waiting for OUT_RAIL");
    return BT::NodeStatus::FAILURE;
  }

  double distance_mm = 0.0;
  rclcpp::Time sample_time;
  uint64_t sequence = 0;
  bool received = false;
  {
    std::lock_guard<std::mutex> lock(tof_mutex_);
    received = tof_received_;
    distance_mm = latest_tof_mm_;
    sample_time = latest_tof_time_;
    sequence = tof_sequence_;
  }

  if (!received || sequence <= start_sequence_) {
    return BT::NodeStatus::RUNNING;
  }
  if ((node_->now() - sample_time).seconds() > tof_stale_sec_) {
    return BT::NodeStatus::RUNNING;
  }
  if (distance_mm < out_tof_mm_) {
    return BT::NodeStatus::RUNNING;
  }

  publishRailState("OUT_RAIL");
  RCLCPP_INFO(node_->get_logger(), "[RailExit] OUT_RAIL: TOF=%.1f mm", distance_mm);
  return BT::NodeStatus::SUCCESS;
}

bool RailExit::timedOut() const
{
  return timeout_sec_ > 0.0 &&
    (node_->now() - start_time_).seconds() >= timeout_sec_;
}

void RailExit::publishRailState(const std::string & state)
{
  std_msgs::msg::String msg;
  msg.data = state;
  rail_state_pub_->publish(msg);
}

}  // namespace theimc_bt_nodes
