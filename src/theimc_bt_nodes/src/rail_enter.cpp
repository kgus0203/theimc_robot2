#include "theimc_bt_nodes/rail_enter.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

namespace theimc_bt_nodes
{

RailEnter::RailEnter(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(xml_tag_name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error(
            "RailEnter: ROS node was not found in BT blackboard");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  tof_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/tof_distance",
    rclcpp::QoS(10),
    std::bind(
      &RailEnter::tofCallback,
      this,
      std::placeholders::_1),
    options);

  rail_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/rail_state",
    rclcpp::QoS(10));
}

BT::PortsList RailEnter::providedPorts()
{
  return {
    BT::InputPort<double>(
      "enter_tof_mm",
      120.0,
      "TOF <= threshold confirms ENTERING"),

    BT::InputPort<double>(
      "on_rail_tof_mm",
      160.0,
      "After ENTERING, TOF >= threshold confirms ON_RAIL"),

    BT::InputPort<double>(
      "timeout_sec",
      0.0,
      "Whole RailEnter timeout. 0 means no timeout"),

    BT::InputPort<double>(
      "tof_stale_sec",
      0.5,
      "Maximum accepted TOF sample age and initial TOF wait"),

    BT::InputPort<int>(
      "enter_confirm_samples",
      2,
      "Consecutive low TOF samples required"),

    BT::InputPort<int>(
      "on_rail_confirm_samples",
      3,
      "Consecutive high TOF samples required")
  };
}

void RailEnter::tofCallback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  const double distance_mm = static_cast<double>(msg->data);

  if (!std::isfinite(distance_mm) || distance_mm < 0.0) {
    return;
  }

  std::lock_guard<std::mutex> lock(tof_mutex_);
  latest_tof_mm_ = distance_mm;
  latest_tof_time_ = node_->now();
  tof_received_ = true;
}

BT::NodeStatus RailEnter::onStart()
{
  getInput("enter_tof_mm", enter_tof_mm_);
  getInput("on_rail_tof_mm", on_rail_tof_mm_);
  getInput("timeout_sec", timeout_sec_);
  getInput("tof_stale_sec", tof_stale_sec_);
  getInput("enter_confirm_samples", enter_confirm_samples_);
  getInput("on_rail_confirm_samples", on_rail_confirm_samples_);

  if (
    enter_tof_mm_ <= 0.0 ||
    on_rail_tof_mm_ <= enter_tof_mm_ ||
    timeout_sec_ < 0.0 ||
    tof_stale_sec_ <= 0.0 ||
    enter_confirm_samples_ < 1 ||
    on_rail_confirm_samples_ < 1)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[RailEnter] Invalid input parameters");

    return BT::NodeStatus::FAILURE;
  }

  phase_ = Phase::WAIT_ENTRY_LOW;
  enter_low_count_ = 0;
  on_rail_high_count_ = 0;

  {
    std::lock_guard<std::mutex> lock(tof_mutex_);
    tof_received_ = false;
  }

  start_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailEnter] START state-only: "
    "wait <= %.1f mm then >= %.1f mm",
    enter_tof_mm_,
    on_rail_tof_mm_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RailEnter::onRunning()
{
  callback_group_executor_.spin_some();

  if (timedOut()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[RailEnter] Timeout");
    return BT::NodeStatus::FAILURE;
  }

  double tof_mm = 0.0;

  if (!getFreshTof(tof_mm)) {
    // RailEnter no longer owns /cmd_vel, so stale/missing TOF must fail
    // instead of allowing DriveUntilRailState to drive forever blindly.
    if ((node_->now() - start_time_).seconds() > tof_stale_sec_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[RailEnter] TOF missing/stale - FAIL for safety");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  switch (phase_) {
    case Phase::WAIT_ENTRY_LOW:
    {
      // Initial normal ground/rail-top high value is intentionally ignored.
      if (tof_mm <= enter_tof_mm_) {
        ++enter_low_count_;
      } else {
        enter_low_count_ = 0;
      }

      if (enter_low_count_ >= enter_confirm_samples_) {
        publishRailState("ENTERING");
        phase_ = Phase::WAIT_ON_RAIL_HIGH;
        on_rail_high_count_ = 0;

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailEnter] state => ENTERING (TOF %.1f mm)",
          tof_mm);
      }

      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_ON_RAIL_HIGH:
    {
      if (tof_mm >= on_rail_tof_mm_) {
        ++on_rail_high_count_;
      } else {
        on_rail_high_count_ = 0;
      }

      if (on_rail_high_count_ >= on_rail_confirm_samples_) {
        publishRailState("ON_RAIL");

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailEnter] state => ON_RAIL (TOF %.1f mm)",
          tof_mm);

        return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::FAILURE;
}

void RailEnter::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[RailEnter] Halted");
}

bool RailEnter::getFreshTof(double & distance_mm)
{
  std::lock_guard<std::mutex> lock(tof_mutex_);

  if (!tof_received_) {
    return false;
  }

  const double age_sec =
    (node_->now() - latest_tof_time_).seconds();

  if (age_sec > tof_stale_sec_) {
    return false;
  }

  distance_mm = latest_tof_mm_;
  return true;
}

bool RailEnter::timedOut() const
{
  if (timeout_sec_ <= 0.0) {
    return false;
  }

  return
    (node_->now() - start_time_).seconds() >= timeout_sec_;
}

void RailEnter::publishRailState(const std::string & state)
{
  std_msgs::msg::String msg;
  msg.data = state;
  rail_state_pub_->publish(msg);
}

}  // namespace theimc_bt_nodes
