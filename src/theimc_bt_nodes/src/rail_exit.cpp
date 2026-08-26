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
    throw std::runtime_error(
            "RailExit: ROS node was not found in BT blackboard");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;

  tof_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/tof_distance",
    rclcpp::QoS(10),
    std::bind(
      &RailExit::tofCallback,
      this,
      std::placeholders::_1),
    subscription_options);

  rail_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/rail_state",
    rclcpp::QoS(10));
}

BT::PortsList RailExit::providedPorts()
{
  return {
    BT::InputPort<double>(
      "exit_tof_mm",
      120.0,
      "TOF <= threshold means rail exit has started"),

    BT::InputPort<double>(
      "out_rail_tof_mm",
      160.0,
      "After EXITING was detected, TOF >= threshold means OUT_RAIL"),

    BT::InputPort<double>(
      "timeout_sec",
      0.0,
      "Whole RailExit timeout. 0 means no timeout"),

    BT::InputPort<double>(
      "tof_stale_sec",
      0.5,
      "Maximum accepted TOF sample age"),

    BT::InputPort<int>(
      "exit_confirm_samples",
      2,
      "Consecutive low TOF samples required for EXITING"),

    BT::InputPort<int>(
      "out_rail_confirm_samples",
      3,
      "Consecutive high TOF samples required for OUT_RAIL")
  };
}

void RailExit::tofCallback(
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

BT::NodeStatus RailExit::onStart()
{
  getInput("exit_tof_mm", exit_tof_mm_);
  getInput("out_rail_tof_mm", out_rail_tof_mm_);
  getInput("timeout_sec", timeout_sec_);
  getInput("tof_stale_sec", tof_stale_sec_);
  getInput("exit_confirm_samples", exit_confirm_samples_);
  getInput("out_rail_confirm_samples", out_rail_confirm_samples_);

  if (
    exit_tof_mm_ <= 0.0 ||
    out_rail_tof_mm_ <= exit_tof_mm_ ||
    timeout_sec_ < 0.0 ||
    tof_stale_sec_ <= 0.0 ||
    exit_confirm_samples_ < 1 ||
    out_rail_confirm_samples_ < 1)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[RailExit] Invalid input parameters");

    return BT::NodeStatus::FAILURE;
  }

  phase_ = Phase::WAIT_EXIT_LOW;

  exit_low_count_ = 0;
  out_rail_high_count_ = 0;

  start_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailExit] START - ignore current high TOF; "
    "wait TOF <= %.1f mm (%d samples), then "
    "TOF >= %.1f mm (%d samples) => OUT_RAIL",
    exit_tof_mm_,
    exit_confirm_samples_,
    out_rail_tof_mm_,
    out_rail_confirm_samples_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RailExit::onRunning()
{
  callback_group_executor_.spin_some();

  if (timedOut()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[RailExit] Timeout");

    return BT::NodeStatus::FAILURE;
  }

  double tof_mm = 0.0;

  if (!getFreshTof(tof_mm)) {
    return BT::NodeStatus::RUNNING;
  }

  switch (phase_) {

    // ------------------------------------------------------------
    // RailExit 시작 시 로봇은 ON_RAIL 상태.
    //
    // 따라서 초기 TOF 160~180 mm는 절대로 OUT_RAIL로 인정하지 않는다.
    // 먼저 후진하면서 100~120 mm 영역을 반드시 통과해야 한다.
    // ------------------------------------------------------------
    case Phase::WAIT_EXIT_LOW:
    {
      if (tof_mm <= exit_tof_mm_) {
        ++exit_low_count_;

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailExit] exit-low candidate: %.1f mm (%d/%d)",
          tof_mm,
          exit_low_count_,
          exit_confirm_samples_);
      } else {
        exit_low_count_ = 0;
      }

      if (exit_low_count_ >= exit_confirm_samples_) {
        publishRailState("EXITING");

        phase_ = Phase::EXITING;
        out_rail_high_count_ = 0;

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailExit] state => EXITING (TOF %.1f mm)",
          tof_mm);
      }

      return BT::NodeStatus::RUNNING;
    }

    // ------------------------------------------------------------
    // 이미 <=120 mm 영역을 통과한 상태.
    //
    // 이제 다시 높은 TOF가 연속해서 확인되면
    // 레일을 완전히 빠져나온 것으로 판단한다.
    // ------------------------------------------------------------
    case Phase::EXITING:
    {
      if (tof_mm >= out_rail_tof_mm_) {
        ++out_rail_high_count_;

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailExit] out-rail candidate: %.1f mm (%d/%d)",
          tof_mm,
          out_rail_high_count_,
          out_rail_confirm_samples_);
      } else {
        out_rail_high_count_ = 0;
      }

      if (out_rail_high_count_ >= out_rail_confirm_samples_) {
        publishRailState("OUT_RAIL");

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailExit] state => OUT_RAIL (TOF %.1f mm)",
          tof_mm);

        return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::FAILURE;
}

void RailExit::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[RailExit] Halted");
}

bool RailExit::getFreshTof(double & distance_mm)
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

bool RailExit::timedOut() const
{
  if (timeout_sec_ <= 0.0) {
    return false;
  }

  return
    (node_->now() - start_time_).seconds() >= timeout_sec_;
}

void RailExit::publishRailState(const std::string & state)
{
  std_msgs::msg::String msg;
  msg.data = state;

  rail_state_pub_->publish(msg);
}

}  // namespace theimc_bt_nodes
