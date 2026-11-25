#include "theimc_bt_nodes/rail_enter.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <chrono>
#include <map>
#include <mutex>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "theimc_bt_nodes/rail_power_feedback.hpp"

namespace theimc_bt_nodes
{
namespace {
// The supplied .hpp is actually a C++ implementation, not a class header.
// Keep the existing installed rail_enter.hpp ABI unchanged by storing the new
// per-instance subscriptions/state here rather than inventing header members.
struct EntryGateState {
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overlap_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
  bool overlap_complete{false};
  bool cmd_seen{false};
  double linear{0.0};
  double angular{0.0};
  std::chrono::steady_clock::time_point cmd_at{};
  std::chrono::steady_clock::time_point enter_started{};
  double finish_timeout_sec{35.0};
  bool prearmed_logged{false};
  bool final_status_ready{false};
};
std::mutex g_entry_gate_mutex;
std::map<const RailEnter*, EntryGateState> g_entry_gates;
}  // namespace

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
  // Completion becomes true only after the driver's 10s overlap,
  // wheel STOP, TOF seating confirmation and Axis1/2 PowerDisable.
  auto overlap_sub = node_->create_subscription<std_msgs::msg::Bool>(
    "/rail/entry_overlap_complete", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
      auto it = g_entry_gates.find(this);
      if (it != g_entry_gates.end()) it->second.overlap_complete = msg->data;
    }, options);

  // Confirm a fresh, straight, forward /cmd_vel BEFORE sending ENTERING.
  // agv_node rejects ENTERING without a current positive straight command.
  auto cmd_sub = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
      auto it = g_entry_gates.find(this);
      if (it == g_entry_gates.end()) return;
      it->second.linear = msg->linear.x;
      it->second.angular = msg->angular.z;
      it->second.cmd_seen = true;
      it->second.cmd_at = std::chrono::steady_clock::now();
    }, options);
  {
    std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
    auto & gate = g_entry_gates[this];
    gate.overlap_sub = std::move(overlap_sub);
    gate.cmd_sub = std::move(cmd_sub);
  }
}

BT::PortsList RailEnter::providedPorts()
{
  return {
    BT::InputPort<double>(
      "enter_tof_mm",
      120.0,
      "TOF <= threshold confirms ENTERING after Axis3 ready"),

    BT::InputPort<double>(
      "on_rail_tof_mm",
      160.0,
      "After ENTERING, TOF >= threshold confirms ON_RAIL"),

    BT::InputPort<double>(
      "timeout_sec",
      0.0,
      "Max seconds waiting for ENTERING only; overlap has its own timeout"),

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
      "Maximum age of /agv/axis_diagnostics ready feedback"),

    BT::InputPort<double>(
      "overlap_timeout_sec", 35.0,
      "Maximum seconds from ENTERING through 10s overlap to final PLC power confirmation")
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
  double overlap_timeout_sec = 35.0;
  getInput("overlap_timeout_sec", overlap_timeout_sec);

  if (
    enter_tof_mm_ <= 0.0 ||
    on_rail_tof_mm_ <= enter_tof_mm_ ||
    timeout_sec_ < 0.0 ||
    power_ready_timeout_sec_ <= 0.0 ||
    power_feedback_stale_sec_ <= 0.0 ||
    !std::isfinite(overlap_timeout_sec) || overlap_timeout_sec <= 0.0 ||
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
  {
    std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
    auto & gate = g_entry_gates[this];
    gate.overlap_complete = false;
    gate.cmd_seen = false;
    gate.linear = 0.0;
    gate.angular = 0.0;
    gate.finish_timeout_sec = overlap_timeout_sec;
    gate.enter_started = {};
    gate.prearmed_logged = false;
    gate.final_status_ready = false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailEnter] START: pre-entry timeout %.1fs, early TOF <= %.1fmm, "
    "on-rail TOF >= %.1fmm; overlap timeout %.1fs",
    timeout_sec_, enter_tof_mm_, on_rail_tof_mm_, overlap_timeout_sec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RailEnter::onRunning()
{
  callback_group_executor_.spin_some();
  const auto now = std::chrono::steady_clock::now();

  // The pre-entry timeout only limits time to ENTERING. It must not cancel
  // a valid 10s drive overlap. A separate finite deadline covers the
  // remainder and preserves the failure/recovery behavior.
  if (phase_ == Phase::WAIT_ENTRY_LOW && timedOut()) {
    RCLCPP_ERROR(node_->get_logger(),
      "[RailEnter] PRE-ENTRY TIMEOUT %.1fs before ENTERING", timeout_sec_);
    return BT::NodeStatus::FAILURE;
  }
  if (phase_ != Phase::WAIT_ENTRY_LOW) {
    std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
    const auto it = g_entry_gates.find(this);
    if (it != g_entry_gates.end() &&
        it->second.enter_started != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(now - it->second.enter_started).count() >
          it->second.finish_timeout_sec) {
      RCLCPP_ERROR(node_->get_logger(),
        "[RailEnter] OVERLAP/HANDOVER TIMEOUT %.1fs after ENTERING",
        it->second.finish_timeout_sec);
      return BT::NodeStatus::FAILURE;
    }
  }

  // ON_RAIL is published only AFTER driver's full overlap-complete feedback.
  // Wait for the verified PLC power configuration before reporting SUCCESS.
  if (phase_ == Phase::WAIT_POWER_READY) {
    const bool diag_fresh = axis_diag_received_ &&
      std::chrono::duration<double>(now - axis_diag_received_at_).count() <= power_feedback_stale_sec_;
    const bool status_fresh = agv_status_received_ &&
      std::chrono::duration<double>(now - agv_status_received_at_).count() <= 2.5;
    bool final_status_ready = false;
    {
      std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
      const auto it = g_entry_gates.find(this);
      final_status_ready = it != g_entry_gates.end() && it->second.final_status_ready;
    }
    const bool ready = diag_fresh && status_fresh && final_status_ready &&
      !axis_ready_[0] && !axis_ready_[1] && axis_ready_[2];
    if (ready) {
      RCLCPP_INFO(node_->get_logger(),
        "[RailEnter] SUCCESS: ON_RAIL / Axis1=OFF Axis2=OFF Axis3=ON");
      return BT::NodeStatus::SUCCESS;
    }
    const double elapsed = std::chrono::duration<double>(now - power_wait_started_).count();
    if (elapsed >= power_ready_timeout_sec_ && !power_timeout_reported_) {
      power_timeout_reported_ = true;
      RCLCPP_ERROR(node_->get_logger(),
        "[RailEnter] ON_RAIL power confirmation delayed %.1fs; waiting until bounded overall timeout",
        elapsed);
    }
    return BT::NodeStatus::RUNNING;
  }

  double tof_mm = 0.0;

  if (!getFreshTof(tof_mm)) {
    // RailEnter no longer owns /cmd_vel, so stale/missing TOF must fail
    // instead of allowing DriveUntilRailState to drive forever blindly.
    // Missing TOF at any stage is unsafe: the whole entry cannot be blind.
    // getFreshTof() protects the timestamp under the TOF mutex.
    // Missing data cannot be used as proof of rail seating.
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
      // Begin before rail impact, but NEVER before pre-arm and a fresh +0.2m/s
      // straight command: driver requires EntryStage::PREPARED + fresh cmd_vel.
      const bool diag_fresh = axis_diag_received_ &&
        std::chrono::duration<double>(now - axis_diag_received_at_).count() <= power_feedback_stale_sec_;
      const bool status_fresh = agv_status_received_ &&
        std::chrono::duration<double>(now - agv_status_received_at_).count() <= 2.5;
      // In PREPARED the driver is READY, wheels ON, rail ON; it publishes its
      // exact transition state only internally, so Axis Ready is a necessary
      // but not sufficient confirmation. Timing is protected by the driver.
      const bool powered = diag_fresh && status_fresh &&
        agv_status_ready_ && axis_ready_[0] && axis_ready_[1] && axis_ready_[2];
      bool fresh_straight_command = false;
      {
        std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
        const auto it = g_entry_gates.find(this);
        if (it != g_entry_gates.end()) {
          const auto & gate = it->second;
          fresh_straight_command = gate.cmd_seen &&
            std::chrono::duration<double>(now - gate.cmd_at).count() <= 0.5 &&
            std::isfinite(gate.linear) && std::isfinite(gate.angular) &&
            gate.linear >= 0.05 && gate.linear <= 0.20 &&
            std::fabs(gate.angular) <= 0.03;
        }
      }
      if (powered && fresh_straight_command && tof_mm <= enter_tof_mm_) {
        ++enter_low_count_;
      } else {
        enter_low_count_ = 0;
      }

      if (enter_low_count_ >= enter_confirm_samples_) {
        {
          std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
          g_entry_gates[this].enter_started = now;
        }
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

      bool complete = false;
      {
        std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
        const auto it = g_entry_gates.find(this);
        complete = it != g_entry_gates.end() && it->second.overlap_complete;
      }
      if (on_rail_high_count_ >= on_rail_confirm_samples_ && complete) {
        publishRailState("ON_RAIL");

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailEnter] state => ON_RAIL (TOF %.1f mm, overlap complete and wheels OFF)",
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
  // For pre-arm, driver reports READY while OUT_RAIL or RAIL_APPROACH.
  // For final confirmation, require the exact ON_RAIL / wheels OFF / rail ON.
  agv_status_ready_ =
    s.find("AGV ADS Driver: READY / rail_state=") != std::string::npos &&
    s.find(" / dual_opt_in=OFF") != std::string::npos;
  {
    std::lock_guard<std::mutex> lock(g_entry_gate_mutex);
    auto it = g_entry_gates.find(this);
    if (it != g_entry_gates.end()) {
      it->second.final_status_ready = agv_status_ready_ &&
        s.find("rail_state=ON_RAIL") != std::string::npos &&
        s.find(" / wheels=DISABLE") != std::string::npos &&
        s.find(" / rail=ENABLE") != std::string::npos;
    }
  }
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