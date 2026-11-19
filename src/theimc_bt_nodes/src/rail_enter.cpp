#include "theimc_bt_nodes/rail_enter.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <chrono>
#include "theimc_bt_nodes/rail_power_feedback.hpp"

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


  axis_diag_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/agv/axis_diagnostics", rclcpp::QoS(10),
    std::bind(&RailEnter::axisDiagnosticsCallback, this, std::placeholders::_1),
    options);
  agv_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/agv_status", rclcpp::QoS(10),
    std::bind(&RailEnter::agvStatusCallback, this, std::placeholders::_1),
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
      "Consecutive high TOF samples required"),

    BT::InputPort<double>(
      "power_ready_timeout_sec", 12.0,
      "Warn after this many seconds waiting for PLC motor power feedback; hold safely"),

    BT::InputPort<double>(
      "power_feedback_stale_sec", 1.0,
      "Maximum age of /agv/axis_diagnostics ready feedback")
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
  getInput("power_ready_timeout_sec", power_ready_timeout_sec_);
  getInput("power_feedback_stale_sec", power_feedback_stale_sec_);

  if (
    enter_tof_mm_ <= 0.0 ||
    on_rail_tof_mm_ <= enter_tof_mm_ ||
    timeout_sec_ < 0.0 ||
    power_ready_timeout_sec_ <= 0.0 ||
    power_feedback_stale_sec_ <= 0.0 ||
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
  axis_diag_received_ = false;
  agv_status_received_ = false;
  agv_status_ready_ = false;
  power_timeout_reported_ = false;

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
  // ON_RAIL/OUT_RAIL has already been announced. Do not recheck TOF or
  // fall into the pre-entry/pre-exit recovery routine while power switches.
  if (phase_ == Phase::WAIT_POWER_READY) {
    const auto now = std::chrono::steady_clock::now();
    const bool diag_fresh = axis_diag_received_ &&
      std::chrono::duration<double>(now - axis_diag_received_at_).count() <= power_feedback_stale_sec_;
    const bool status_fresh = agv_status_received_ &&
      std::chrono::duration<double>(now - agv_status_received_at_).count() <= 2.5;
    const bool ready = diag_fresh && status_fresh && agv_status_ready_ && !axis_ready_[0] && !axis_ready_[1] && axis_ready_[2];
    if (ready) {
      RCLCPP_INFO(node_->get_logger(), "[RailEnter] PLC power ready: Axis1=OFF Axis2=OFF Axis3=ON");
      return BT::NodeStatus::SUCCESS;
    }
    const double elapsed = std::chrono::duration<double>(now - power_wait_started_).count();
    if (elapsed >= power_ready_timeout_sec_ && !power_timeout_reported_) {
      power_timeout_reported_ = true;
      RCLCPP_ERROR(node_->get_logger(),
        "[RailEnter] POWER HANDOVER TIMEOUT (%.1fs). HOLDING RUNNING safely. "
        "Check /agv/axis_diagnostics and /agv/recovery_diagnostics; NO next motion.", elapsed);
    }
    return BT::NodeStatus::RUNNING;
  }


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

        // Power handover is handled by agv_node upon /rail_state.
        // A BT SUCCESS here would allow motion before Axis Ready feedback.
        phase_ = Phase::WAIT_POWER_READY;
        power_wait_started_ = std::chrono::steady_clock::now();
        axis_diag_received_ = false;  // discard a pre-transition snapshot
        agv_status_received_ = false;
        return BT::NodeStatus::RUNNING;
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

void RailEnter::axisDiagnosticsCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::array<bool, 3> ready{};
  if (!parse_axis_ready(msg->data, ready)) {
    axis_diag_received_ = false;
    return;
  }
  axis_ready_ = ready;
  axis_diag_received_at_ = std::chrono::steady_clock::now();
  axis_diag_received_ = true;
}

void RailEnter::agvStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Require the driver READY report as well as axis feedback, so a faulted
  // or reconnected driver cannot be mistaken for a completed handover.
  const std::string &s = msg->data;
  agv_status_ready_ = s.find("AGV ADS Driver: READY / rail_state=ON_ENABLE") != std::string::npos &&
    s.find(" / wheels=DISABLE") != std::string::npos &&
    s.find(" / rail=ENABLE") != std::string::npos &&
    s.find(" / dual_opt_in=OFF") != std::string::npos;
  agv_status_received_ = true;
  agv_status_received_at_ = std::chrono::steady_clock::now();
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