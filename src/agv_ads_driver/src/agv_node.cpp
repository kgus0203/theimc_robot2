#include <array>
#include <chrono>
#include <csignal>  // SIGPIPE handling: ADS peer may close socket during TwinCAT restart
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <functional>
#include <cstring>
#include <memory>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

// Vendor recovery policy (2026-11-05):
// 1) Normal direction reversal must STOP first and wait for standstill.
// 2) If StopMove is rejected/BUSY, immediately PowerDisable the affected axis.
// 3) Keep all motion latched, inspect the resulting alarm, then Reset.
// 4) Re-enable the required drive group only after all axes are confirmed stopped.
// PLC/drive-independent E-stop and watchdog remain mandatory.
// Vendor-provided Beckhoff wrapper API.
#include "TcMCApi.h"

using namespace std::chrono_literals;

// Vendor TcMCApi.h is intentionally UNCHANGED.  Its StopMove/MoveVelocity
// functions return an ADS method result, NOT the PLC FB Done/Busy state.
// Read the same PLC *_Status methods over our own read-only ADS port.
// All calls are made on the ROS executor thread; no extra PLC command thread.
class PlcFbStatusReader
{
public:
  PlcFbStatusReader() = default;
  PlcFbStatusReader(const PlcFbStatusReader &) = delete;
  PlcFbStatusReader & operator=(const PlcFbStatusReader &) = delete;
  ~PlcFbStatusReader() { close(); }

  bool is_open() const { return port_ > 0; }

  bool open(const AmsNetId & remote_net_id)
  {
    if (port_ > 0) return true;
    addr_ = {};
    addr_.netId = remote_net_id;
    addr_.port = AMSPORT_R0_PLC_TC3;
    port_ = AdsPortOpenEx();
    if (port_ <= 0) { port_ = 0; return false; }
    // Status port only; vendor Move/Power API has its own connection.
    if (AdsSyncSetTimeoutEx(port_, 500) != 0) { close(); return false; }
    if (!acquire_handle("Stop_Status", stop_handle_) ||
        !acquire_handle("MoveVelocity_Status", move_handle_) ||
        !acquire_handle("Stop", stop_command_handle_) ||
        !acquire_handle("Reset", reset_command_handle_)) {
      close();
      return false;
    }
    return true;
  }

  void close()
  {
    if (port_ > 0) {
      release_handle(stop_handle_);
      release_handle(move_handle_);
      release_handle(stop_command_handle_);
      release_handle(reset_command_handle_);
      AdsPortCloseEx(port_);
    }
    stop_handle_ = 0;
    move_handle_ = 0;
    stop_command_handle_ = 0;
    reset_command_handle_ = 0;
    port_ = 0;
  }

  bool read_stop(int axis, FunctionStatus & out) {
    return read_status(stop_handle_, axis, out);
  }
  bool read_move(int axis, FunctionStatus & out) {
    return read_status(move_handle_, axis, out);
  }

  // One ADS RPC only: unlike vendor StopMove(), do NOT synchronously wait
  // for MC_Stop.Done here.  The timer verifies FB and NC state afterwards.
  // Operator-triggered stale-job recovery only.  This does not move/re-arm.
  bool send_stop_once(int axis, double ramp_ms, double jerk, int32_t & plc_result)
  {
    plc_result = UNKNOWN_ERR_TcMCFUN;
    if (port_ <= 0 || stop_command_handle_ == 0 || axis < 1 || axis > 3) return false;
#pragma pack(push, 1)
    struct StopArgs {
      uint32_t nIndex;
      double lRampTime;
      double lJerk;
    } args{};
#pragma pack(pop)
    static_assert(sizeof(StopArgs) == 20, "Stop ADS argument layout mismatch");
    args.nIndex = uint32_t{1} << (axis - 1);
    args.lRampTime = ramp_ms;
    args.lJerk = jerk;
    uint32_t bytes_read = 0;
    const long rc = AdsSyncReadWriteReqEx2(
      port_, &addr_, ADSIGRP_SYM_VALBYHND, stop_command_handle_,
      sizeof(plc_result), &plc_result,
      sizeof(args), &args, &bytes_read);
    return rc == 0 && bytes_read == sizeof(plc_result) && plc_result == 0;
  }

  // A Reset edge can release a *completed* MC_Stop lock.  Never use it
  // to override an active motion job, and never infer success from ADS ACK.
  bool send_reset_once(int axis, int32_t & plc_result)
  {
    plc_result = UNKNOWN_ERR_TcMCFUN;
    if (port_ <= 0 || reset_command_handle_ == 0 || axis < 1 || axis > 3) return false;
    const uint32_t axis_mask = uint32_t{1} << (axis - 1);
    uint32_t bytes_read = 0;
    const long rc = AdsSyncReadWriteReqEx2(
      port_, &addr_, ADSIGRP_SYM_VALBYHND, reset_command_handle_,
      sizeof(plc_result), &plc_result,
      sizeof(axis_mask), const_cast<uint32_t *>(&axis_mask), &bytes_read);
    return rc == 0 && bytes_read == sizeof(plc_result) && plc_result == 0;
  }

private:
  bool acquire_handle(const char * method, uint32_t & handle)
  {
    const std::string name = std::string("MotionHandler.fbAxisHandler#") + method;
    uint32_t cb = 0;
    handle = 0;
    const long rc = AdsSyncReadWriteReqEx2(
      port_, &addr_, ADSIGRP_SYM_HNDBYNAME, 0,
      sizeof(handle), &handle,
      static_cast<uint32_t>(name.size() + 1),
      const_cast<char *>(name.c_str()), &cb);
    return rc == 0 && cb == sizeof(handle) && handle != 0;
  }

  void release_handle(uint32_t & handle)
  {
    if (!handle || port_ <= 0) return;
    (void)AdsSyncWriteReqEx(
      port_, &addr_, ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle), &handle);
    handle = 0;
  }

  bool read_status(uint32_t handle, int axis, FunctionStatus & out)
  {
    out = {};
    if (port_ <= 0 || handle == 0 || axis < 1 || axis > 3) return false;
#pragma pack(push, 1)
    struct PlcResult {
      bool bBusy;
      bool bError;
      bool bDone;
      uint32_t nError;
      int32_t nResult;
    } value{};
#pragma pack(pop)
    static_assert(sizeof(PlcResult) == 11, "PLC status ADS size mismatch");
    uint32_t axis_mask = (uint32_t{1} << (axis - 1));
    uint32_t cb = 0;
    const long rc = AdsSyncReadWriteReqEx2(
      port_, &addr_, ADSIGRP_SYM_VALBYHND, handle,
      sizeof(value), &value, sizeof(axis_mask),
      &axis_mask, &cb);
    if (rc != 0 || cb != sizeof(value) || value.nResult < 0) return false;
    out.bBusy = value.bBusy;
    out.bError = value.bError;
    out.bDone = value.bDone;
    out.nError = value.nError;
    return true;
  }

  long port_{0};
  AmsAddr addr_{};
  uint32_t stop_handle_{0};
  uint32_t stop_command_handle_{0};
  uint32_t reset_command_handle_{0};
  uint32_t move_handle_{0};
};

class AgvAdsDriver : public rclcpp::Node
{
public:
  AgvAdsDriver()
  : Node("agv_ads_driver")
  {
    // ============================================================
    // Parameters
    // ============================================================
    this->declare_parameter<bool>("mock_mode", true);

    // Confirmed by vendor:
    // Axis 1 / 2 = driving motors, Axis 3 = rail motor.
    // Left/right of Axis 1/2 may swap depending on assembly.
    this->declare_parameter<int>("left_wheel_axis", 1);
    this->declare_parameter<int>("right_wheel_axis", 2);
    this->declare_parameter<int>("rail_axis", 3);

    // If PLC positive direction is opposite to ROS forward direction,
    // change the corresponding sign to -1.0.
    this->declare_parameter<double>("left_wheel_sign", 1.0);
    this->declare_parameter<double>("right_wheel_sign", 1.0);
    this->declare_parameter<double>("rail_sign", 1.0);

    // IMPORTANT: set the real center-to-center distance between
    // left/right drive wheels before field driving.
    this->declare_parameter<double>("wheel_separation_m", 0.930);

    // Measured driving-wheel nominal diameter = 8 in = 203.2 mm.
    // Only used to convert PLC-provided linear wheel speed to joint angular speed
    // for /joint_states; NEVER used to rescale ADS wheel velocity/position.
    this->declare_parameter<double>("wheel_diameter_m", 0.2032);

    // Vendor confirmed API units:
    // MoveVelocity velocity = mm/s
    // AxisStatus.lActVelocity = mm/s
    // AxisStatus.lActPosition = mm
    // PLC already accounts for wheel/rail wheel sizes (rail wheel: 130 mm),
    // gearing and encoder scaling. No second diameter/gear conversion here.
    this->declare_parameter<double>("velocity_scale", 1000.0);  // m/s -> mm/s

    // Vendor confirmed:
    // ramp = milliseconds
    // jerk = 0..100 (% S-curve amount)
    this->declare_parameter<double>("wheel_ramp_ms", 1000.0);
    this->declare_parameter<double>("rail_ramp_ms", 1000.0);
    // Rail Axis3 requires an explicit STOP before reversing direction.
    // After feedback reaches near-zero, keep STOP state for this additional hold time.
    this->declare_parameter<double>("rail_direction_stop_hold_ms", 200.0);
    this->declare_parameter<double>("jerk_percent", 100.0);

    this->declare_parameter<double>("cmd_vel_timeout_sec", 10.0);
    this->declare_parameter<double>("rail_cmd_timeout_sec", 10.0);

    // An axis bError must NOT cause StopMove(). Vendor PLC can stay BUSY
    // if MC_Stop is requested during an axis fault. Reset is separated from
    // motion re-arm. Enable one-shot automatic Reset only after site validation.
    this->declare_parameter<bool>("auto_reset_axis_fault", false);
    // Vendor instruction: if STOP cannot be accepted, disable the affected drive immediately.
    this->declare_parameter<bool>("disable_on_stop_failure", true);
    // A fault Reset is NOT permission to re-enable a disabled servo.
    this->declare_parameter<bool>("require_power_restore_approval", true);
    // Continuous PLC standstill/FB-idle verification before releasing Reset latch.
    this->declare_parameter<double>("reset_standstill_verify_sec", 3.0);
    this->declare_parameter<double>("reset_recovery_timeout_sec", 5.0);

    // Avoid repeatedly sending nearly-identical MoveVelocity requests.
    this->declare_parameter<double>("wheel_command_epsilon_mps", 0.005);

    // PLC-feedback wheel balance for straight/reverse/pivot/curves.
    // Equalize measured speed magnitude when target magnitudes are equal;
    // on curves equalize actual/target tracking ratios.
    this->declare_parameter<bool>("wheel_balance_enable", true);
    this->declare_parameter<double>("wheel_balance_kp", 0.20);
    this->declare_parameter<double>("wheel_balance_max_correction_mps", 0.010);
    this->declare_parameter<double>("wheel_balance_deadband_mps", 0.005);
    this->declare_parameter<double>("wheel_balance_min_speed_mps", 0.050);
    this->declare_parameter<double>("wheel_balance_angular_threshold_rps", 0.030);
    this->declare_parameter<double>("wheel_balance_filter_alpha", 0.15);
    // Minimum time after BOTH accepted Axis1/2 MoveVelocity commands
    // before computing another feedback-driven correction.
    this->declare_parameter<double>("wheel_balance_settle_sec", 1.50);
    this->declare_parameter<double>("wheel_balance_stable_hold_sec", 1.0);
    this->declare_parameter<double>("wheel_balance_release_rate_mps2", 0.005);
    this->declare_parameter<double>("wheel_balance_reverse_hold_sec", 0.25);

    // IMU yaw heading hold (straight) and yaw-rate tracking (turning).
    // Conservative, sampled updates to avoid Beckhoff MoveVelocity chatter.
    this->declare_parameter<bool>("imu_correction_enable", true);
    this->declare_parameter<double>("imu_timeout_sec", 0.5);
    this->declare_parameter<double>("imu_heading_kp", 0.30);
    this->declare_parameter<double>("imu_heading_kd", 0.1);
    this->declare_parameter<double>("imu_turn_kp", 0.3);
    this->declare_parameter<double>("imu_max_angular_correction_rps", 0.05);
    // Limit the change per completed IMU control update, not every 20Hz timer tick.
    this->declare_parameter<double>("imu_max_correction_step_rps", 0.015);
    this->declare_parameter<double>("imu_correction_interval_sec", 2.0);
    this->declare_parameter<double>("imu_straight_angular_threshold_rps", 0.03);
    this->declare_parameter<double>("imu_min_wheel_speed_mps", 0.08);
    // Site-verified IMU yaw axis is opposite ROS positive yaw; invert both
    // quaternion-derived yaw and angular_velocity.z consistently.
    // A launch/YAML override takes precedence over this default.
    this->declare_parameter<double>("imu_yaw_sign", -1.0);
    // Limit Beckhoff MoveVelocity ADS calls. Nav2 may publish at 20+ Hz, but
    // the PLC motion method must not be called for every small velocity update.
    // STOP is one-shot per accepted MoveVelocity, never retried after BUSY.
    this->declare_parameter<double>("wheel_move_command_min_interval_ms", 100.0);
    // Conservative inter-command delay: move ramp + margin. This is NOT
    // proof that MC function blocks finished; PLC Method status is preferable.
    this->declare_parameter<double>("ads_command_settle_extra_ms", 150.0);
    // Suppress STOP at near-zero speed ONLY when no PLC job is active.
    // HasJob=1 at zero speed may be a stalled axis with retained command.
    this->declare_parameter<double>("stop_suppress_velocity_mps", 0.002);
    this->declare_parameter<double>("axis_notification_max_age_sec", 1.5);
    // BUSY stop interlock: block new wheel motion until both wheels stop.
    this->declare_parameter<double>("wheel_stop_pending_timeout_sec", 4.0);
    this->declare_parameter<double>("rail_command_epsilon_mps", 0.001);

    // Vendor says PowerEnable is required once before motion.
    this->declare_parameter<bool>("power_enable_on_start", true);
    // Strict read-only mode: do not transmit any ADS Power/Move/Stop/Reset
    // methods, even if other ROS nodes call power/reset services.
    // PLC or other ADS clients remain outside this node's control.
    this->declare_parameter<bool>("diagnostic_read_only", false);
    // For investigation of delayed PLC bNotMoving feedback. This ONLY extends
    // the wait: it never bypasses bNotMoving, HasJob, or STOP FB interlocks.
    this->declare_parameter<double>("startup_power_transition_timeout_sec", 5.0);
    this->declare_parameter<bool>("power_disable_on_shutdown", true);
    // Give each drive its own verified shutdown.  A stale job on Axis3 must
    // not prevent powering down a safely stopped Axis1/2.
    this->declare_parameter<double>("shutdown_poweroff_timeout_sec", 7.0);

    // EXPERIMENTAL: independent Wheel(1/2) + Rail(3) control. Disabled by
    // default because the vendor originally required mutually exclusive
    // drive power. Enable ONLY after mechanical/PLC safety validation.
    // In this mode startup is still Wheel ON / Rail OFF; enable Rail with Y
    // while all axes are confirmed stopped. Mode changes never auto-cycle
    // drive power. Per-axis STOP/fault/feedback interlocks remain active.
    this->declare_parameter<bool>("allow_simultaneous_drive", false);
    this->declare_parameter<double>("dual_wheel_axis_max_mps", 0.05);
    this->declare_parameter<double>("dual_rail_max_mps", 0.10);

    // Normal MoveVelocity/StopMove command logs can become very noisy
    // because Nav2 may change /cmd_vel frequently. Keep them off by default.
    // Errors and important state transitions are still logged.
    this->declare_parameter<bool>("log_motion_commands", false);

    // Axis diagnostic monitor.
    // Prints command/actual speed, Ready, Error, ErrorCode for Axis1/2/3.
    // This does not change the motion behavior.
    this->declare_parameter<bool>("log_axis_monitor", false);
    this->declare_parameter<double>("axis_monitor_period_sec", 0.5);
    // In read-only diagnostics mode, do not request an ADS snapshot at 20 Hz.
    // Only reduces traffic and ROS executor blocking probability; an ADS
    // synchronous request may still exceed this period due to network loss.
    this->declare_parameter<double>("diagnostic_ads_poll_period_sec", 0.5);
    this->declare_parameter<double>("ads_slow_read_warn_ms", 100.0);
    // Connection recovery is transport-only: NEVER auto PowerEnable or resume motion.
    this->declare_parameter<bool>("ads_auto_reconnect", true);
    this->declare_parameter<double>("ads_reconnect_interval_sec", 5.0);
    this->declare_parameter<int>("ads_disconnect_failure_count", 3);

    // Safe STOP handling.
    //
    // Vendor sample sequence is:
    //   StopMove() -> wait until AxisStatus.bNotMoving -> PowerDisable()
    //
    // Axis2 has returned 0x98110708 (ADSERR_DEVICE_BUSY) when StopMove is
    // requested. Never PowerDisable a wheel that is still moving. When BUSY
    // occurs, keep power enabled and inhibit new motion. NEVER retry STOP
    // automatically; confirm the drive/PLC state with the vendor.
    // Deprecated: kept for launch compatibility; STOP retries are disabled.
    this->declare_parameter<double>("stop_retry_interval_ms", 1000.0);
    this->declare_parameter<double>("stop_wait_timeout_sec", 4.0);
    this->declare_parameter<double>("stopped_velocity_threshold_mps", 0.01);

    // Beckhoff environmental sensors.
    // Vendor PLC symbol: Sensor_Modbus.SensorInfo
    //
    // IMPORTANT:
    // We use the vendor ADS notification path instead of adding a synchronous
    // GetSensorData() read to the 20 Hz motor-control loop.  A slow ADS read
    // must not delay wheel/rail watchdog and STOP handling.
    this->declare_parameter<bool>("enable_environment_sensor", true);
    this->declare_parameter<double>("environment_publish_period_sec", 1.0);
    this->declare_parameter<bool>("log_environment_sensor", false);

    mock_mode_ = this->get_parameter("mock_mode").as_bool();

    left_wheel_axis_ = static_cast<int>(this->get_parameter("left_wheel_axis").as_int());
    right_wheel_axis_ = static_cast<int>(this->get_parameter("right_wheel_axis").as_int());
    rail_axis_ = static_cast<int>(this->get_parameter("rail_axis").as_int());

    left_wheel_sign_ = normalize_sign(this->get_parameter("left_wheel_sign").as_double());
    right_wheel_sign_ = normalize_sign(this->get_parameter("right_wheel_sign").as_double());
    rail_sign_ = normalize_sign(this->get_parameter("rail_sign").as_double());

    wheel_separation_m_ = this->get_parameter("wheel_separation_m").as_double();
    wheel_diameter_m_ = this->get_parameter("wheel_diameter_m").as_double();
    velocity_scale_ = this->get_parameter("velocity_scale").as_double();

    wheel_ramp_ms_ = this->get_parameter("wheel_ramp_ms").as_double();
    rail_ramp_ms_ = this->get_parameter("rail_ramp_ms").as_double();
    rail_direction_stop_hold_ms_ =
      this->get_parameter("rail_direction_stop_hold_ms").as_double();
    jerk_percent_ = clamp_jerk(this->get_parameter("jerk_percent").as_double());

    cmd_vel_timeout_sec_ = this->get_parameter("cmd_vel_timeout_sec").as_double();
    rail_cmd_timeout_sec_ = this->get_parameter("rail_cmd_timeout_sec").as_double();
    auto_reset_axis_fault_ = this->get_parameter("auto_reset_axis_fault").as_bool();
    disable_on_stop_failure_ = this->get_parameter("disable_on_stop_failure").as_bool();
    require_power_restore_approval_ = this->get_parameter("require_power_restore_approval").as_bool();
    reset_standstill_verify_sec_ = this->get_parameter("reset_standstill_verify_sec").as_double();
    reset_recovery_timeout_sec_ = this->get_parameter("reset_recovery_timeout_sec").as_double();

    wheel_command_epsilon_mps_ =
      std::fabs(this->get_parameter("wheel_command_epsilon_mps").as_double());

    wheel_balance_enable_ = this->get_parameter("wheel_balance_enable").as_bool();
    wheel_balance_kp_ = this->get_parameter("wheel_balance_kp").as_double();
    wheel_balance_max_correction_mps_ =
      std::fabs(this->get_parameter("wheel_balance_max_correction_mps").as_double());
    wheel_balance_deadband_mps_ =
      std::fabs(this->get_parameter("wheel_balance_deadband_mps").as_double());
    wheel_balance_min_speed_mps_ =
      std::fabs(this->get_parameter("wheel_balance_min_speed_mps").as_double());
    wheel_balance_angular_threshold_rps_ =
      std::fabs(this->get_parameter("wheel_balance_angular_threshold_rps").as_double());
    wheel_balance_filter_alpha_ =
      this->get_parameter("wheel_balance_filter_alpha").as_double();
    wheel_balance_settle_sec_ =
      this->get_parameter("wheel_balance_settle_sec").as_double();
    wheel_balance_stable_hold_sec_ =
      this->get_parameter("wheel_balance_stable_hold_sec").as_double();
    wheel_balance_release_rate_mps2_ =
      this->get_parameter("wheel_balance_release_rate_mps2").as_double();
    wheel_balance_reverse_hold_sec_ =
      this->get_parameter("wheel_balance_reverse_hold_sec").as_double();
    imu_correction_enable_ = this->get_parameter("imu_correction_enable").as_bool();
    imu_timeout_sec_ = this->get_parameter("imu_timeout_sec").as_double();
    imu_heading_kp_ = this->get_parameter("imu_heading_kp").as_double();
    imu_heading_kd_ = this->get_parameter("imu_heading_kd").as_double();
    imu_turn_kp_ = this->get_parameter("imu_turn_kp").as_double();
    imu_max_angular_correction_rps_ =
      this->get_parameter("imu_max_angular_correction_rps").as_double();
    imu_max_correction_step_rps_ =
      this->get_parameter("imu_max_correction_step_rps").as_double();
    imu_correction_interval_sec_ =
      this->get_parameter("imu_correction_interval_sec").as_double();
    imu_straight_angular_threshold_rps_ =
      this->get_parameter("imu_straight_angular_threshold_rps").as_double();
    imu_min_wheel_speed_mps_ =
      this->get_parameter("imu_min_wheel_speed_mps").as_double();
    imu_yaw_sign_ = normalize_sign(this->get_parameter("imu_yaw_sign").as_double());
    wheel_move_command_min_interval_ms_ =
      this->get_parameter("wheel_move_command_min_interval_ms").as_double();
    ads_command_settle_extra_ms_ = this->get_parameter("ads_command_settle_extra_ms").as_double();
    stop_suppress_velocity_mps_ = this->get_parameter("stop_suppress_velocity_mps").as_double();
    axis_notification_max_age_sec_ = this->get_parameter("axis_notification_max_age_sec").as_double();
    wheel_stop_pending_timeout_sec_ =
      this->get_parameter("wheel_stop_pending_timeout_sec").as_double();
    rail_command_epsilon_mps_ =
      std::fabs(this->get_parameter("rail_command_epsilon_mps").as_double());

    power_enable_on_start_ = this->get_parameter("power_enable_on_start").as_bool();
    diagnostic_read_only_ = this->get_parameter("diagnostic_read_only").as_bool();
    startup_power_transition_timeout_sec_ =
      this->get_parameter("startup_power_transition_timeout_sec").as_double();
    power_disable_on_shutdown_ = this->get_parameter("power_disable_on_shutdown").as_bool();
    shutdown_poweroff_timeout_sec_ =
      this->get_parameter("shutdown_poweroff_timeout_sec").as_double();
    allow_simultaneous_drive_ = this->get_parameter("allow_simultaneous_drive").as_bool();
    dual_wheel_axis_max_mps_ = this->get_parameter("dual_wheel_axis_max_mps").as_double();
    dual_rail_max_mps_ = this->get_parameter("dual_rail_max_mps").as_double();
    log_motion_commands_ = this->get_parameter("log_motion_commands").as_bool();

    log_axis_monitor_ = this->get_parameter("log_axis_monitor").as_bool();
    axis_monitor_period_sec_ =
      this->get_parameter("axis_monitor_period_sec").as_double();
    diagnostic_ads_poll_period_sec_ =
      this->get_parameter("diagnostic_ads_poll_period_sec").as_double();
    ads_slow_read_warn_ms_ =
      this->get_parameter("ads_slow_read_warn_ms").as_double();
    ads_auto_reconnect_ = this->get_parameter("ads_auto_reconnect").as_bool();
    ads_reconnect_interval_sec_ = this->get_parameter("ads_reconnect_interval_sec").as_double();
    ads_disconnect_failure_count_ = static_cast<int>(
      this->get_parameter("ads_disconnect_failure_count").as_int());

    stop_retry_interval_ms_ =
      this->get_parameter("stop_retry_interval_ms").as_double();
    stop_wait_timeout_sec_ =
      this->get_parameter("stop_wait_timeout_sec").as_double();
    stopped_velocity_threshold_mps_ =
      this->get_parameter("stopped_velocity_threshold_mps").as_double();

    enable_environment_sensor_ =
      this->get_parameter("enable_environment_sensor").as_bool();
    environment_publish_period_sec_ =
      this->get_parameter("environment_publish_period_sec").as_double();
    log_environment_sensor_ =
      this->get_parameter("log_environment_sensor").as_bool();

    if (!std::isfinite(ads_reconnect_interval_sec_) || ads_reconnect_interval_sec_ < 1.0 ||
        ads_reconnect_interval_sec_ > 120.0 || ads_disconnect_failure_count_ < 2 ||
        ads_disconnect_failure_count_ > 20) {
      throw std::runtime_error("invalid ADS reconnect retry/failure parameters");
    }
    if (!std::isfinite(reset_standstill_verify_sec_) || reset_standstill_verify_sec_ < 1.0 ||
        reset_standstill_verify_sec_ > 10.0 ||
        !std::isfinite(reset_recovery_timeout_sec_) ||
        reset_recovery_timeout_sec_ <= reset_standstill_verify_sec_ ||
        reset_recovery_timeout_sec_ > 30.0) {
      throw std::runtime_error("invalid Reset verification/timeout parameters");
    }
    validate_parameters();
    // Log values actually received by this node, not the source or installed
    // launch file contents.  This helps detect stale install/share copies.
    RCLCPP_WARN(this->get_logger(),
      "[AGV EFFECTIVE PARAMETERS] diagnostic_read_only=%d power_enable_on_start=%d "
      "power_disable_on_shutdown=%d dual_opt_in=%d "
      "startup_timeout=%.1fs cmd_vel_timeout=%.1fs rail_timeout=%.1fs "
      "diag_poll=%.2fs ads_slow_warning=%.0fms",
      diagnostic_read_only_ ? 1 : 0,
      power_enable_on_start_ ? 1 : 0,
      power_disable_on_shutdown_ ? 1 : 0,
      allow_simultaneous_drive_ ? 1 : 0,
      startup_power_transition_timeout_sec_,
      cmd_vel_timeout_sec_, rail_cmd_timeout_sec_,
      diagnostic_ads_poll_period_sec_, ads_slow_read_warn_ms_);
    if (allow_simultaneous_drive_) {
      RCLCPP_WARN(this->get_logger(),
        "[DUAL DRIVE EXPERIMENTAL] Wheels and Rail may be simultaneously enabled "
        "and commanded ONLY by explicit operator power requests. "
        "Vendor mechanical interlock must be validated. Wheel axis <= %.3f m/s, "
        "rail <= %.3f m/s when both powered. Independent PLC E-stop required.",
        dual_wheel_axis_max_mps_, dual_rail_max_mps_);
    }
    if (!mock_mode_ && cmd_vel_timeout_sec_ > 10.0) {
      RCLCPP_WARN(this->get_logger(),
        "[SAFETY] /cmd_vel timeout %.1fs: diagnostic-only setting. "
        "Real robot must use an independent PLC command-loss watchdog/E-stop.",
        cmd_vel_timeout_sec_);
    }

    // ============================================================
    // ROS subscriptions
    // ============================================================
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&AgvAdsDriver::cmd_vel_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&AgvAdsDriver::imu_callback, this, std::placeholders::_1));

    rail_cmd_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/rail/cmd_speed", 10,
      std::bind(&AgvAdsDriver::rail_cmd_speed_callback, this, std::placeholders::_1));

    rail_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/rail_state", 10,
      std::bind(&AgvAdsDriver::rail_state_callback, this, std::placeholders::_1));

    // ============================================================
    // ROS publishers
    // ============================================================
    wheel_speed_feedback_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/wheel/speed_feedback", 10);

    rail_speed_feedback_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/rail/speed_feedback", 10);

    wheel_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(
      "/wheel/odom", 10);

    joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    status_pub_ =
      this->create_publisher<std_msgs::msg::String>(
      "/agv_status", 10);

    axis_diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/agv/axis_diagnostics", 10);

    // Environmental sensor topics from Beckhoff PLC / Modbus sensor block.
    // /environment/data order:
    //   [0] solar radiation [W/m^2]
    //   [1] temperature [degC]
    //   [2] humidity [%]
    //   [3] CO2 [ppm]
    environment_data_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/environment/data", 10);

    environment_solar_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(
      "/environment/solar_radiation", 10);

    environment_temperature_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(
      "/environment/temperature", 10);

    environment_humidity_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(
      "/environment/humidity", 10);

    environment_co2_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(
      "/environment/co2", 10);

    approve_power_restore_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/agv/approve_power_restore",
      std::bind(&AgvAdsDriver::approve_power_restore_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    reset_faults_srv_ =
      this->create_service<std_srvs::srv::Trigger>(
      "/agv/reset_faults",
      std::bind(
        &AgvAdsDriver::reset_faults_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    // Read-only recovery explanation: works even while motion is latched.
    recovery_diagnostics_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/agv/recovery_diagnostics",
      std::bind(&AgvAdsDriver::recovery_diagnostics_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    recover_stale_jobs_srv_ =
      this->create_service<std_srvs::srv::Trigger>(
      "/agv/recover_stale_jobs",
      std::bind(
        &AgvAdsDriver::recover_stale_jobs_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    approve_ads_recovery_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/agv/approve_ads_recovery",
      std::bind(&AgvAdsDriver::approve_ads_recovery_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    wheel_power_srv_ =
      this->create_service<std_srvs::srv::SetBool>(
      "/agv/wheel_power",
      std::bind(
        &AgvAdsDriver::wheel_power_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    rail_power_srv_ =
      this->create_service<std_srvs::srv::SetBool>(
      "/agv/rail_power",
      std::bind(
        &AgvAdsDriver::rail_power_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    last_cmd_vel_time_ = this->now();
    last_rail_cmd_time_ = this->now();
    last_cmd_vel_steady_ = std::chrono::steady_clock::now();
    last_rail_cmd_steady_ = last_cmd_vel_steady_;
    last_ads_reconnect_tp_ = last_cmd_vel_steady_;
    last_feedback_time_ = this->now();
    last_axis_monitor_time_ = this->now();
    last_environment_publish_time_ = this->now();

    // ============================================================
    // Beckhoff connection / initial PowerEnable
    // ============================================================
    if (!mock_mode_) {
      connect_and_initialize();
      if (!ads_connected_ || !plc_fb_reader_.is_open()) {
        mark_ads_connection_lost("initial ADS connection/PLC FB handle not ready");
      }
    } else {
      // Mock starts in the same logical state as OUT_RAIL.
      motion_ready_ = true;
      wheel_axes_enabled_ = true;
      rail_axis_enabled_ = false;
    }

    // 20 Hz control + feedback loop.
    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&AgvAdsDriver::timer_callback, this));

    /*RCLCPP_INFO(
      this->get_logger(),
      "AGV ADS Driver initialized: mock_mode=%s, left_axis=%d, right_axis=%d, rail_axis=%d",
      mock_mode_ ? "true" : "false",
      left_wheel_axis_, right_wheel_axis_, rail_axis_);
    */

    /*RCLCPP_INFO(
      this->get_logger(),
      "Vendor units: velocity=mm/s, position=mm, ramp=ms, jerk=%.1f%%",
      jerk_percent_);
    */

    RCLCPP_DEBUG(
      this->get_logger(),
      "Axis1/2 left-right assignment and direction signs depend on assembly. "
      "Verify at low speed before normal operation.");

    RCLCPP_DEBUG(
      this->get_logger(),
      "Rail Axis3 mechanical parameters are not field-calibrated yet. "
      "Rail speed/position physical accuracy must be verified on-site.");

    /*RCLCPP_INFO(
      this->get_logger(),
      "Axis monitor: %s, period=%.2f sec",
      log_axis_monitor_ ? "ENABLED" : "DISABLED",
      axis_monitor_period_sec_);
    */

    /*RCLCPP_INFO(
      this->get_logger(),
      "Safe STOP policy: retry=%.0f ms, wait_timeout=%.1f sec, "
      "stopped_threshold=%.3f m/s; PowerDisable only after bNotMoving + near-zero velocity",
      stop_retry_interval_ms_,
      stop_wait_timeout_sec_,
      stopped_velocity_threshold_mps_);
    */

    /*RCLCPP_INFO(
      this->get_logger(),
      "Environment sensor: %s, publish_period=%.2f sec, "
      "ADS symbol=Sensor_Modbus.SensorInfo",
      enable_environment_sensor_ ? "ENABLED" : "DISABLED",
      environment_publish_period_sec_);
    */

    /*RCLCPP_INFO(
      this->get_logger(),
      "Power interlock policy: OUT_RAIL/MOVING_TO_RAIL/RAIL_APPROACH/ENTERING => "
      "Wheels ENABLE, Rail DISABLE; ON_RAIL/WORKING/EXITING => "
      "Wheels DISABLE, Rail ENABLE.");
    */
  }

  ~AgvAdsDriver() override
  {
    shutting_down_ = true;
    stop_environment_polling();
    motion_ready_ = false;
    if (timer_) {
      timer_->cancel();
    }

    if (!mock_mode_ && ads_connected_) {
      clear_motion_requests();
      // ROS context can already be shut down here. ADS calls do not depend
      // on ROS scheduling, but this is a bounded best-effort cleanup only.
      if (diagnostic_read_only_) {
        RCLCPP_WARN(this->get_logger(),
          "[SHUTDOWN] diagnostic_read_only=true: no STOP/PowerDisable writes.");
      } else if (power_disable_on_shutdown_ && !ads_recovery_required_) {
        // Called while the ADS session is still alive.  This must never
        // be replaced with an unconditional PowerDisable of moving axes.
        shutdown_stop_then_poweroff();
      } else {
        RCLCPP_WARN(this->get_logger(),
          "[SHUTDOWN] No ADS Power/Stop writes: transport recovery required or shutdown poweroff disabled. "
          "Verify drives by independent PLC/hardware means.");
      }
      plc_fb_reader_.close();
      Disconnect();
      ads_connected_ = false;
      RCLCPP_INFO(this->get_logger(), "[SHUTDOWN] Beckhoff ADS disconnected.");
    }
  }

private:
  // PLC axis IDs are 1-based; arrays are 0-based. Fail loudly on invalid IDs.
  static std::size_t axis_index(int axis_num)
  {
    if (axis_num < 1 || axis_num > 3) {
      throw std::out_of_range("PLC axis number out of range (1..3)");
    }
    return static_cast<std::size_t>(axis_num - 1);
  }

  static constexpr double kPowerFeedbackTimeoutSec = 5.0;
  static constexpr double kStaleRecoveryTimeoutSec = 8.0;
  static constexpr double kStaleRecoveryMaxVelocityMps = 0.001;
  static constexpr double kStaleRecoveryStandstillSec = 1.0;

  static double normalize_sign(double value)
  {
    return value < 0.0 ? -1.0 : 1.0;
  }

  static double clamp_jerk(double value)
  {
    if (!std::isfinite(value)) {
      return 100.0;
    }
    if (value < 0.0) {
      return 0.0;
    }
    if (value > 100.0) {
      return 100.0;
    }
    return value;
  }

  void validate_parameters()
  {
    const auto valid_axis = [](int axis) {
      return axis >= 1 && axis <= 3;
    };

    if (!valid_axis(left_wheel_axis_) ||
        !valid_axis(right_wheel_axis_) ||
        !valid_axis(rail_axis_))
    {
      throw std::runtime_error(
              "left_wheel_axis/right_wheel_axis/rail_axis must be in range 1..3");
    }

    if (left_wheel_axis_ == right_wheel_axis_ ||
        left_wheel_axis_ == rail_axis_ ||
        right_wheel_axis_ == rail_axis_)
    {
      throw std::runtime_error(
              "left/right/rail axis numbers must be different");
    }

    // Invalid watchdog settings must never silently disable timeout protection.
    if (!std::isfinite(cmd_vel_timeout_sec_) || cmd_vel_timeout_sec_ <= 0.0 ||
        !std::isfinite(rail_cmd_timeout_sec_) || rail_cmd_timeout_sec_ <= 0.0) {
      throw std::runtime_error("wheel/rail command watchdog timeouts must be finite and > 0");
    }

    if (!std::isfinite(wheel_separation_m_) || wheel_separation_m_ <= 0.0) {
      throw std::runtime_error("wheel_separation_m must be > 0");
    }

    if (!std::isfinite(wheel_diameter_m_) || wheel_diameter_m_ <= 0.0) {
      throw std::runtime_error("wheel_diameter_m must be > 0");
    }

    if (!std::isfinite(velocity_scale_) || velocity_scale_ <= 0.0) {
      throw std::runtime_error("velocity_scale must be > 0");
    }

    if (!std::isfinite(rail_direction_stop_hold_ms_) ||
        rail_direction_stop_hold_ms_ < 0.0)
    {
      throw std::runtime_error("rail_direction_stop_hold_ms must be >= 0");
    }

    if (!std::isfinite(dual_wheel_axis_max_mps_) || dual_wheel_axis_max_mps_ <= 0.0 ||
        !std::isfinite(dual_rail_max_mps_) || dual_rail_max_mps_ <= 0.0) {
      throw std::runtime_error("dual speed limits must be finite and > 0");
    }
    if (!std::isfinite(axis_monitor_period_sec_) ||
        axis_monitor_period_sec_ <= 0.0)
    {
      throw std::runtime_error("axis_monitor_period_sec must be > 0");
    }
    if (!std::isfinite(diagnostic_ads_poll_period_sec_) ||
        diagnostic_ads_poll_period_sec_ < 0.05 ||
        diagnostic_ads_poll_period_sec_ > 30.0) {
      throw std::runtime_error("diagnostic_ads_poll_period_sec must be within 0.05..30s");
    }
    if (!std::isfinite(ads_slow_read_warn_ms_) ||
        ads_slow_read_warn_ms_ < 1.0 || ads_slow_read_warn_ms_ > 30000.0) {
      throw std::runtime_error("ads_slow_read_warn_ms must be within 1..30000ms");
    }

    if (!std::isfinite(ads_command_settle_extra_ms_) ||
        ads_command_settle_extra_ms_ < 0.0) {
      throw std::runtime_error("ads_command_settle_extra_ms must be >= 0");
    }
    if (!std::isfinite(axis_notification_max_age_sec_) ||
        axis_notification_max_age_sec_ <= 0.0) {
      throw std::runtime_error("axis_notification_max_age_sec must be > 0");
    }
    if (!std::isfinite(stop_suppress_velocity_mps_) ||
        stop_suppress_velocity_mps_ < 0.0 ||
        stop_suppress_velocity_mps_ >= stopped_velocity_threshold_mps_) {
      throw std::runtime_error(
        "stop_suppress_velocity_mps must be >= 0 and < stopped_velocity_threshold_mps");
    }
    if (!std::isfinite(wheel_move_command_min_interval_ms_) ||
        wheel_move_command_min_interval_ms_ < 0.0)
    {
      throw std::runtime_error("wheel_move_command_min_interval_ms must be >= 0");
    }

    if (!std::isfinite(imu_timeout_sec_) || imu_timeout_sec_ <= 0.0 ||
        !std::isfinite(imu_heading_kp_) || imu_heading_kp_ < 0.0 ||
        !std::isfinite(imu_heading_kd_) || imu_heading_kd_ < 0.0 ||
        !std::isfinite(imu_turn_kp_) || imu_turn_kp_ < 0.0 ||
        !std::isfinite(imu_max_angular_correction_rps_) ||
        imu_max_angular_correction_rps_ < 0.0 || imu_max_angular_correction_rps_ > 0.2 ||
        !std::isfinite(imu_max_correction_step_rps_) ||
        imu_max_correction_step_rps_ <= 0.0 || imu_max_correction_step_rps_ > 0.2 ||
        !std::isfinite(imu_correction_interval_sec_) || imu_correction_interval_sec_ < 1.5 ||
        !std::isfinite(imu_straight_angular_threshold_rps_) ||
        imu_straight_angular_threshold_rps_ < 0.0 ||
        !std::isfinite(imu_min_wheel_speed_mps_) || imu_min_wheel_speed_mps_ <= 0.0) {
      throw std::runtime_error("invalid IMU steering parameters");
    }
    if (!std::isfinite(wheel_balance_kp_) || wheel_balance_kp_ < 0.0 ||
        !std::isfinite(wheel_balance_max_correction_mps_) ||
        wheel_balance_max_correction_mps_ < 0.0 ||
        !std::isfinite(wheel_balance_deadband_mps_) ||
        wheel_balance_deadband_mps_ < 0.0 ||
        !std::isfinite(wheel_balance_min_speed_mps_) ||
        wheel_balance_min_speed_mps_ < 0.0 ||
        !std::isfinite(wheel_balance_angular_threshold_rps_) ||
        wheel_balance_angular_threshold_rps_ < 0.0 ||
        !std::isfinite(wheel_balance_filter_alpha_) ||
        wheel_balance_filter_alpha_ <= 0.0 ||
        wheel_balance_filter_alpha_ > 1.0 ||
        !std::isfinite(wheel_balance_settle_sec_) ||
        wheel_balance_settle_sec_ < 0.5 || wheel_balance_settle_sec_ > 10.0 ||
        !std::isfinite(wheel_balance_stable_hold_sec_) ||
        wheel_balance_stable_hold_sec_ < 0.2 || wheel_balance_stable_hold_sec_ > 10.0 ||
        !std::isfinite(wheel_balance_release_rate_mps2_) ||
        wheel_balance_release_rate_mps2_ <= 0.0 || wheel_balance_release_rate_mps2_ > 0.1 ||
        !std::isfinite(wheel_balance_reverse_hold_sec_) ||
        wheel_balance_reverse_hold_sec_ < 0.05 || wheel_balance_reverse_hold_sec_ > 3.0)
    {
      throw std::runtime_error("invalid wheel balance parameters");
    }
    if (!std::isfinite(wheel_stop_pending_timeout_sec_) ||
        wheel_stop_pending_timeout_sec_ <= 0.0)
    {
      throw std::runtime_error("wheel_stop_pending_timeout_sec must be > 0");
    }

    if (!std::isfinite(stop_retry_interval_ms_) ||
        stop_retry_interval_ms_ < 50.0)
    {
      throw std::runtime_error("stop_retry_interval_ms must be >= 50");
    }

    if (!std::isfinite(startup_power_transition_timeout_sec_) ||
        startup_power_transition_timeout_sec_ < 1.0 ||
        startup_power_transition_timeout_sec_ > 120.0) {
      throw std::runtime_error(
        "startup_power_transition_timeout_sec must be within 1..120 seconds");
    }

    if (!std::isfinite(shutdown_poweroff_timeout_sec_) ||
        shutdown_poweroff_timeout_sec_ < 1.0 ||
        shutdown_poweroff_timeout_sec_ > 12.0) {
      throw std::runtime_error("shutdown_poweroff_timeout_sec must be 1..12s");
    }
    if (!std::isfinite(stop_wait_timeout_sec_) ||
        stop_wait_timeout_sec_ <= 0.0)
    {
      throw std::runtime_error("stop_wait_timeout_sec must be > 0");
    }

    if (!std::isfinite(stopped_velocity_threshold_mps_) ||
        stopped_velocity_threshold_mps_ <= 0.0)
    {
      throw std::runtime_error("stopped_velocity_threshold_mps must be > 0");
    }

    if (!std::isfinite(environment_publish_period_sec_) ||
        environment_publish_period_sec_ <= 0.0)
    {
      throw std::runtime_error("environment_publish_period_sec must be > 0");
    }
  }

  // ============================================================
  // Safe STOP helpers
  // ============================================================
  enum class StopRequestResult
  {
    ACCEPTED,
    BUSY,
    RATE_LIMITED,
    ALREADY_STOPPED,
    ALREADY_REQUESTED,
    WAITING_FOR_PREVIOUS,
    FAILED
  };

  static bool is_ads_device_busy(int32_t result)
  {
    return static_cast<uint32_t>(result) == 0x98110708U;
  }

  void clear_motion_requests()
  {
    requested_linear_x_mps_ = 0.0;
    requested_angular_z_rps_ = 0.0;
    requested_rail_speed_mps_ = 0.0;

    left_pending_target_mps_ = 0.0;
    right_pending_target_mps_ = 0.0;
    rail_pending_target_mps_ = 0.0;

    left_waiting_for_stop_ = false;
    right_waiting_for_stop_ = false;
    rail_waiting_for_stop_ = false;
    rail_stop_hold_active_ = false;
    rail_stop_sequence_active_ = false;
    rail_stop_tx_sent_ = false;
    rail_stop_retry_wait_active_ = false;
    wheel_direction_change_active_ = false;
    wheel_pair_stable_initialized_ = false;
    wheel_direction_pending_left_mps_ = 0.0;
    wheel_direction_pending_right_mps_ = 0.0;
    reset_wheel_balance();
    reset_imu_correction();
  }

  void enter_safe_stop(const std::string & reason)
  {
    clear_motion_requests();
    motion_ready_ = false;
    safe_stop_active_ = true;

    if (!safe_stop_latched_) {
      safe_stop_latched_ = true;
      safe_stop_reason_ = reason;
      // Do not clear the one-shot STOP gate here: that would resend STOP.
      RCLCPP_ERROR(
        this->get_logger(),
        "[SAFE STOP LATCH] %s. All new motion commands are inhibited until "
        "/agv/reset_faults succeeds.",
        reason.c_str());
    }
  }

  void process_safe_stop(const std::array<AxisStatus, 3> & status)
  {
    if (!safe_stop_latched_ || !safe_stop_active_ || fault_latched_ ||
        has_any_fault(status)) {
      // Axis fault uses a separate Reset-only recovery path. NEVER queue
      // another StopMove() from the generic SAFE STOP handler while faulted.
      return;
    }

    const auto & left = status[axis_index(left_wheel_axis_)];
    const auto & right = status[axis_index(right_wheel_axis_)];
    const auto & rail = status[axis_index(rail_axis_)];

    const bool left_stopped = axis_is_physically_stopped(left, left_wheel_sign_);
    const bool right_stopped = axis_is_physically_stopped(right, right_wheel_sign_);
    const bool rail_stopped = axis_is_physically_stopped(rail, rail_sign_);

    if (!left_stopped) {
      (void)request_axis_stop(left_wheel_axis_, wheel_ramp_ms_, "safe-stop latch");
    }
    if (!right_stopped) {
      (void)request_axis_stop(right_wheel_axis_, wheel_ramp_ms_, "safe-stop latch");
    }
    if (!rail_stopped) {
      (void)request_axis_stop(rail_axis_, rail_ramp_ms_, "safe-stop latch");
    }

    if (left_stopped && right_stopped && rail_stopped) {
      safe_stop_active_ = false;
      applied_left_mps_ = 0.0;
      applied_right_mps_ = 0.0;
      applied_rail_mps_ = 0.0;
      RCLCPP_ERROR(
        this->get_logger(),
        "[SAFE STOP LATCH] Physical stop confirmed. Motion remains locked. "
        "Call /agv/reset_faults to re-arm motion.");
    }
  }

  // Each call queries the PLC Function Block, not just the ADS transport.
  // Never assume a successful ADS method invocation means the FB is Done.
  bool read_stop_status(int axis, FunctionStatus & out)
  {
    out = {};
    const int32_t rc = plc_fb_reader_.read_stop(axis, out) ? NO_ERR_TcMCFUN : UNKNOWN_ERR_TcMCFUN;
    if (rc != NO_ERR_TcMCFUN) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[PLC STATUS] Axis%d Stop_Status ADS failed: 0x%X", axis,
        static_cast<unsigned int>(rc));
      return false;
    }
    return true;
  }

  bool read_move_status(int axis, FunctionStatus & out)
  {
    out = {};
    const int32_t rc = plc_fb_reader_.read_move(axis, out) ? NO_ERR_TcMCFUN : UNKNOWN_ERR_TcMCFUN;
    if (rc != NO_ERR_TcMCFUN) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[PLC STATUS] Axis%d MoveVelocity_Status ADS failed: 0x%X", axis,
        static_cast<unsigned int>(rc));
      return false;
    }
    return true;
  }

  // Normal idle: no PLC job, no Stop lock, no Move FB in progress.
  // Used ONLY when the desired power mode already matches. This does not
  // authorize PowerDisable while bNotMoving is false.
  bool plc_idle_for_existing_power(const std::array<AxisStatus, 3> & status)
  {
    for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & s = status[axis_index(axis)];
      if (s.bError || s.bHasJob || !s.bNotMoving ||
          !std::isfinite(s.lActVelocity) ||
          std::fabs(s.lActVelocity / velocity_scale_) > stop_suppress_velocity_mps_) {
        RCLCPP_DEBUG(this->get_logger(),
          "[PLC IDLE] Axis%d not idle: HasJob=%d NM=%d speed=%.4f m/s",
          axis, s.bHasJob ? 1 : 0, s.bNotMoving ? 1 : 0,
          s.lActVelocity / velocity_scale_);
        return false;
      }
      if (!s.bControlLoopClosed) {
        // A disabled axis has no active drive controller.  Some PLC
        // implementations reject Function Block status queries on disabled
        // axes; there must still be no HasJob, velocity, or error.
        continue;
      }
      FunctionStatus stop{}, move{};
      if (!read_stop_status(axis, stop) || !read_move_status(axis, move)) return false;
      if (stop.bError || stop.bBusy || move.bError ||
          (!s.bNotMoving && move.bBusy)) {
        RCLCPP_DEBUG(this->get_logger(),
          "[PLC IDLE] Axis%d FB: STOP busy=%d done=%d err=%d(0x%X), "
          "MOVE busy=%d done=%d err=%d(0x%X)",
          axis, stop.bBusy, stop.bDone, stop.bError,
          static_cast<unsigned int>(stop.nError),
          move.bBusy, move.bDone, move.bError,
          static_cast<unsigned int>(move.nError));
        return false;
      }
    }
    return true;
  }

  // MC_MoveVelocity may stay Busy/Active during valid continuous motion.
  // A new speed may supersede it only after its minimum ramp/settle window
  // and the *old* commanded speed has been measured stably (or FB Done).
  // The last accepted target, not the new incoming target, is compared.
  bool previous_command_window_elapsed(int axis, double ramp_ms)
  {
    if (axis < 1 || axis > 3) return false;
    const auto idx = static_cast<std::size_t>(axis);
    if (!last_accepted_move_initialized_[idx]) return true;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      now - last_accepted_move_tp_[idx]).count();
    if (elapsed_ms < ramp_ms + ads_command_settle_extra_ms_) return false;

    FunctionStatus st{};
    if (!read_move_status(axis, st)) return false;
    if (st.bError) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[PLC MOVE ERROR] Axis%d code=0x%X", axis,
        static_cast<unsigned int>(st.nError));
      enter_safe_stop(std::string("PLC MoveVelocity_Status error Axis") +
        std::to_string(axis));
      safe_stop_active_ = false; // Do not blindly STOP a faulted FB.
      return false;
    }
    if (!st.bBusy || st.bDone) {
      move_feedback_stable_initialized_[idx] = false;
      return true;
    }

    // Busy=1 alone is not a fault. Reconcile with *actual* motion feedback.
    std::array<AxisStatus, 3> axes{};
    if (!read_axis_status(axes) || has_any_fault(axes)) return false;
    const auto & feedback = axes[idx - 1];
    const double sign = axis == left_wheel_axis_ ? left_wheel_sign_ :
      (axis == right_wheel_axis_ ? right_wheel_sign_ : rail_sign_);
    const double actual = feedback.lActVelocity / velocity_scale_ * sign;
    const double expected = last_accepted_target_mps_[idx];
    const double tolerance = std::max(0.005, std::fabs(expected) * 0.05);
    if (!feedback.bControlLoopClosed || !std::isfinite(actual) ||
        std::fabs(actual - expected) > tolerance) {
      move_feedback_stable_initialized_[idx] = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[PLC MOVE WAIT] Axis%d Busy=1 Done=%d ACT=%+.4f TARGET=%+.4f m/s",
        axis, st.bDone ? 1 : 0, actual, expected);
      return false;
    }
    if (!move_feedback_stable_initialized_[idx]) {
      move_feedback_stable_since_[idx] = now;
      move_feedback_stable_initialized_[idx] = true;
      return false;
    }
    // Require repeated matching measurements over 250 ms, not one sample.
    return std::chrono::duration<double, std::milli>(
      now - move_feedback_stable_since_[idx]).count() >= 250.0;
  }

  // An MC_Stop FB may return to its idle state (Busy=0, Done=0,
  // Error=0) after the Execute edge is released.  Requiring Done=1
  // permanently therefore deadlocks the next MoveVelocity.  However,
  // Busy=0 alone is NOT proof that the mechanical axis is stationary:
  // require fresh PLC axis feedback and a short, continuous standstill.
  bool previous_stop_window_elapsed(int axis, double stop_ramp_ms)
  {
    if (axis < 1 || axis > 3) return false;
    const auto idx = static_cast<std::size_t>(axis);
    if (!last_stop_tx_initialized_[idx]) return true;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      now - last_stop_tx_tp_[idx]).count();
    if (elapsed_ms < stop_ramp_ms + ads_command_settle_extra_ms_) {
      stop_idle_confirm_initialized_[idx] = false;
      return false;
    }

    FunctionStatus st{};
    if (!read_stop_status(axis, st)) {
      stop_idle_confirm_initialized_[idx] = false;
      return false;
    }

    std::array<AxisStatus, 3> statuses{};
    if (!read_axis_status(statuses) || has_any_fault(statuses)) {
      stop_idle_confirm_initialized_[idx] = false;
      return false;
    }
    const auto & feedback = statuses[idx - 1];
    const double actual_mps = feedback.lActVelocity / velocity_scale_;
    const bool confirmed_standstill =
      !feedback.bError && !feedback.bHasJob && feedback.bNotMoving &&
      !feedback.bMovingFw && !feedback.bMovingBw &&
      std::isfinite(actual_mps) &&
      std::fabs(actual_mps) <= stopped_velocity_threshold_mps_;

    // Done may have already fallen to zero on FB reset, so it is
    // informational here.  Both Busy=0 and verified axis standstill are
    // mandatory, even if Done=1 (do not trust a stale Done sample).
    if (st.bError || st.bBusy || !confirmed_standstill) {
      stop_idle_confirm_initialized_[idx] = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[PLC STOP WAIT] Axis%d Busy=%d Done=%d Error=%d(0x%X) "
        "NM=%d Job=%d FW=%d BW=%d actual=%+.4f m/s; no new command",
        axis, st.bBusy ? 1 : 0, st.bDone ? 1 : 0, st.bError ? 1 : 0,
        static_cast<unsigned int>(st.nError),
        feedback.bNotMoving ? 1 : 0, feedback.bHasJob ? 1 : 0,
        feedback.bMovingFw ? 1 : 0, feedback.bMovingBw ? 1 : 0, actual_mps);
      return false;
    }

    // Avoid passing a momentary/stale zero-speed sample.  The control
    // loop must observe the STOP FB idle and axis stopped for >=200ms.
    constexpr double kStopIdleStableMs = 200.0;
    if (!stop_idle_confirm_initialized_[idx]) {
      stop_idle_confirm_since_[idx] = now;
      stop_idle_confirm_initialized_[idx] = true;
      return false;
    }
    if (std::chrono::duration<double, std::milli>(
        now - stop_idle_confirm_since_[idx]).count() < kStopIdleStableMs) {
      return false;
    }
    return true;
  }

  bool all_previous_stops_settled()
  {
    return previous_stop_window_elapsed(left_wheel_axis_, wheel_ramp_ms_) &&
      previous_stop_window_elapsed(right_wheel_axis_, wheel_ramp_ms_) &&
      previous_stop_window_elapsed(rail_axis_, rail_ramp_ms_);
  }

  // Used ONLY for power handover / confirmed standstill, not for new MOVE.
  // MC_MoveVelocity Busy can remain asserted during continuous motion; it
  // cannot substitute for physical standstill and released STOP FB.
  bool all_previous_moves_settled()
  {
    std::array<AxisStatus, 3> status{};
    if (!read_axis_status(status) || has_any_fault(status) ||
        !all_axes_physically_stopped(status)) return false;
    for (const int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & s = status[axis_index(axis)];
      if (s.bHasJob) return false;
      if (!s.bControlLoopClosed) continue;
      FunctionStatus stop{};
      if (!read_stop_status(axis, stop) || stop.bError || stop.bBusy) {
        return false;
      }
    }
    return true;
  }

  void note_move_accepted(int axis, double accepted_target_mps)
  {
    if (axis < 1 || axis > 3) {return;}
    const auto idx = static_cast<std::size_t>(axis);
    last_accepted_move_tp_[idx] = std::chrono::steady_clock::now();
    last_accepted_move_initialized_[idx] = true;
    last_accepted_target_mps_[idx] = accepted_target_mps;
    move_feedback_stable_initialized_[idx] = false;
    // Only an actual new, accepted non-zero MOVE re-arms the one-STOP gate.
    stop_attempt_initialized_[idx] = false;
    // The previous STOP was settled before this MOVE was admitted; avoid
    // treating its old timestamp as a pending STOP in subsequent cycles.
    last_stop_tx_initialized_[idx] = false;
    stop_idle_confirm_initialized_[idx] = false;
  }

  // Vendor-directed escalation: if STOP itself cannot be accepted, remove
  // drive enable from that axis immediately, then latch all ROS motion.
  // This is intentionally NOT used for normal stopping; it is only the fallback
  // after StopMove returns BUSY/failure. Recovery requires Reset + standstill + PowerEnable.
  void disable_axis_after_stop_failure(int axis, const char * context, int32_t stop_result)
  {
    if (!disable_on_stop_failure_ || diagnostic_read_only_ || !ads_connected_ ||
        axis < 1 || axis > 3) {
      return;
    }

    clear_motion_requests();
    motion_ready_ = false;
    safe_stop_active_ = false;  // do not send another StopMove from generic safe-stop
    safe_stop_latched_ = true;
    safe_stop_reason_ = std::string("STOP failed -> PowerDisable Axis") + std::to_string(axis);

    RCLCPP_ERROR(this->get_logger(),
      "[VENDOR STOP FALLBACK] Axis%d StopMove failed/BUSY (0x%X, %s). "
      "Immediately sending PowerDisable; ALL new motion remains inhibited until Reset/re-arm.",
      axis, static_cast<unsigned int>(stop_result), context);

    const auto axis_id = static_cast<std::size_t>(axis);
    if (fallback_disable_attempted_[axis_id]) return; // one-shot; no ADS spam
    fallback_disable_attempted_[axis_id] = true;
    const int32_t disable_result = PowerDisable(static_cast<uint32_t>(axis));
    axis_snapshot_valid_ = false;
    if (disable_result != NO_ERR_TcMCFUN) {
      RCLCPP_ERROR(this->get_logger(),
        "[VENDOR STOP FALLBACK] Axis%d PowerDisable rejected: 0x%X. "
        "Use hardware E-stop / PLC intervention; do not command motion.",
        axis, static_cast<unsigned int>(disable_result));
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "[VENDOR STOP FALLBACK] Axis%d PowerDisable submitted. "
        "Inspect alarm code, then call /agv/reset_faults after the machine is stationary.", axis);
    }
  }

  // IMPORTANT: NEVER call StopMove on a stationary axis, and NEVER resend a
  // StopMove (whether it succeeded or returned BUSY) for the same move.
  StopRequestResult request_axis_stop(
    int axis,
    double ramp_ms,
    const char * context)
  {
    if (diagnostic_read_only_) return StopRequestResult::FAILED;
    if (axis < 1 || axis > 3 || !ads_connected_ || fault_latched_) {
      return StopRequestResult::FAILED;
    }
    const std::size_t idx = static_cast<std::size_t>(axis);
    std::array<AxisStatus, 3> status{};
    if (!read_axis_status(status) || has_any_fault(status)) {
      // No command can be safely derived from missing/faulted feedback.
      return StopRequestResult::FAILED;
    }
    const auto & axis_status = status[idx - 1];
    const double actual_mps = std::fabs(axis_status.lActVelocity / velocity_scale_);
    const bool not_moving = axis_status.bNotMoving;

    // A blocked wheel can report zero actual speed while the PLC still owns
    // an active velocity job. Do not equate zero encoder speed with STOP.
    if (not_moving && !axis_status.bHasJob &&
        !axis_status.bMovingFw && !axis_status.bMovingBw &&
        actual_mps <= stopped_velocity_threshold_mps_) {
      return StopRequestResult::ALREADY_STOPPED;
    }
    if (stop_attempt_initialized_[idx]) {
      return StopRequestResult::ALREADY_REQUESTED;
    }
    // Zero velocity with HasJob=1 means a commanded movement can resume when
    // an obstacle is removed. Allow ONE STOP request for that exact case.
    // If there is no PLC job, do not blindly STOP an ambiguous stationary axis.
    if (actual_mps <= stop_suppress_velocity_mps_ && !axis_status.bHasJob) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[STOP HELD] Axis%d near-zero speed, HasJob=0, NotMoving=%d; "
        "no blind STOP (%s)", axis, not_moving ? 1 : 0, context);
      return StopRequestResult::WAITING_FOR_PREVIOUS;
    }
    // A near-zero encoder speed with HasJob=1 is NOT safe standstill.
    // Do not attempt to bypass the PLC Stop FB or blindly cut servo torque.
    // The FB state below determines whether a single STOP can be dispatched.
    const bool stalled_job =
      actual_mps <= stop_suppress_velocity_mps_ && axis_status.bHasJob;
    // STOP cancels continuous MoveVelocity.  A running MOVE FB can remain
    // Busy indefinitely; DO NOT wait for ramp+settle or MOVE Busy=0 here.
    // The PLC STOP FB state and the per-motion one-shot latch are the guards.
    // A previous STOP can still be locked in the PLC even without this run's
    // one-shot flag (e.g. after ROS restart). Do not dispatch another STOP.
    FunctionStatus plc_stop{};
    const bool stop_readable = read_stop_status(axis, plc_stop);
    if (!stop_readable || plc_stop.bBusy || plc_stop.bError) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "[STOP BLOCKED] Axis%d stalled=%d speed=%.5f HasJob=%d NM=%d "
        "axis_error=%d axis_code=0x%X STOP_read=%d busy=%d error=%d fb_code=0x%X; "
        "no duplicate STOP, check PLC/independent safe stop",
        axis, stalled_job ? 1 : 0, actual_mps,
        axis_status.bHasJob ? 1 : 0, axis_status.bNotMoving ? 1 : 0,
        axis_status.bError ? 1 : 0,
        static_cast<unsigned int>(axis_status.nErrorCode),
        stop_readable ? 1 : 0, plc_stop.bBusy ? 1 : 0,
        plc_stop.bError ? 1 : 0, static_cast<unsigned int>(plc_stop.nError));
      // A retained velocity job with a locked STOP FB is not a reason to
      // keep sending STOP. Freeze ROS motion exactly once and let a single
      // Reset request monitor job release without rebooting TwinCAT.
      if (stalled_job && stop_readable && (plc_stop.bBusy || plc_stop.bError)) {
        enter_safe_stop(std::string("Axis") + std::to_string(axis) +
          " retained NC job / STOP FB busy");
        safe_stop_active_ = false;
      }
      return StopRequestResult::WAITING_FOR_PREVIOUS;
    }
    if (stalled_job) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "[STALLED JOB] Axis%d speed=%.5f HasJob=1 axis_code=0x%X; "
        "STOP FB idle, sending only ONE guarded STOP", axis, actual_mps,
        static_cast<unsigned int>(axis_status.nErrorCode));
    }

    // One attempt per accepted move: set BEFORE the API call to ensure that
    // a callback/timer re-entry cannot cause another STOP.
    stop_attempt_initialized_[idx] = true;
    last_stop_tx_initialized_[idx] = true;
    stop_idle_confirm_initialized_[idx] = false;
    last_stop_tx_tp_[idx] = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(this->get_logger(),
      "[STOP TX ONCE] Axis%d context=%s measured=%.4f m/s",
      axis, context, actual_mps);
    const int32_t result = StopMove(
      static_cast<uint32_t>(axis), ramp_ms, jerk_percent_);
    if (result == NO_ERR_TcMCFUN) {
      return StopRequestResult::ACCEPTED;
    }

    // Vendor instruction (2026-11-05): do not keep retrying a STOP that did
    // not work. Disable that axis immediately, then recover through alarm Reset.
    if (is_ads_device_busy(result)) {
      RCLCPP_ERROR(this->get_logger(),
        "[STOP BUSY -> DISABLE] Axis%d context=%s: 0x98110708.", axis, context);
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "[STOP FAILED -> DISABLE] Axis%d context=%s result=0x%X",
        axis, context, static_cast<unsigned int>(result));
    }
    if (!shutting_down_) {
      disable_axis_after_stop_failure(axis, context, result);
    }
    return is_ads_device_busy(result) ? StopRequestResult::BUSY
                                      : StopRequestResult::FAILED;
  }

  bool axis_is_physically_stopped(const AxisStatus & status, double sign) const
  {
    const double actual_mps =
      (status.lActVelocity / velocity_scale_) * sign;

    // Follow the vendor example: PowerDisable only after bNotMoving.
    // Also require measured velocity to be near zero as a second safety check.
    return
      status.bNotMoving &&
      std::fabs(actual_mps) <= stopped_velocity_threshold_mps_;
  }

  bool all_axes_physically_stopped(
    const std::array<AxisStatus, 3> & status) const
  {
    return
      axis_is_physically_stopped(
        status[axis_index(left_wheel_axis_)],
        left_wheel_sign_) &&
      axis_is_physically_stopped(
        status[axis_index(right_wheel_axis_)],
        right_wheel_sign_) &&
      axis_is_physically_stopped(
        status[axis_index(rail_axis_)],
        rail_sign_);
  }

  // Ctrl+C / normal node destruction: STOP if moving, then disable each
  // axis INDEPENDENTLY after verified NC standstill.  Never power-disable an
  // axis whose job, direction or MC_Stop state remains unresolved.
  // No Reset and no auto re-arm is performed here.
  void shutdown_stop_then_poweroff()
  {
    clear_motion_requests();
    axis_snapshot_valid_ = false;
    const auto timeout = std::max(1.0, shutdown_poweroff_timeout_sec_);
    const auto started = std::chrono::steady_clock::now();
    std::array<bool, 4> verified_off{{false, false, false, false}};
    std::array<bool, 4> disable_sent{{false, false, false, false}};
    std::array<bool, 4> stable_initialized{{false, false, false, false}};
    std::array<std::chrono::steady_clock::time_point, 4> stable_since{};
    std::array<bool, 4> stop_logged{{false, false, false, false}};
    const std::array<int, 3> axes{{left_wheel_axis_, right_wheel_axis_, rail_axis_}};
    RCLCPP_WARN(this->get_logger(),
      "[SHUTDOWN BEGIN] Request STOP, then verified Power OFF for Axis1/2/3 "
      "(timeout %.1fs). Hardware E-stop remains independent.", timeout);

    while (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count() < timeout) {
      // Force a fresh ADS snapshot: never use a cached zero-speed sample to
      // authorize a drive power transition.
      axis_snapshot_valid_ = false;
      std::array<AxisStatus, 3> state{};
      if (!read_axis_status(state)) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      bool all_verified = true;
      for (const int axis : axes) {
        if (axis < 1 || axis > 3) {
          all_verified = false;
          continue;
        }
        const auto idx = static_cast<std::size_t>(axis);
        const auto & s = state[idx - 1];
        if (verified_off[idx]) continue;
        if (!s.bControlLoopClosed) {
          verified_off[idx] = true;
          RCLCPP_INFO(this->get_logger(),
            "[SHUTDOWN OFF CONFIRMED] Axis%d Ready=0%s", axis,
            disable_sent[idx] ? " after PowerDisable" : " (already OFF)");
          continue;
        }
        all_verified = false;
        if (disable_sent[idx]) {
          // PowerDisable command was sent.  Wait for actual READY=0.
          continue;
        }

        const double vel_mps = std::fabs(s.lActVelocity / velocity_scale_);
        const bool nc_stopped = !s.bError && s.bNotMoving &&
          !s.bHasJob && !s.bMovingFw && !s.bMovingBw &&
          std::isfinite(vel_mps) && vel_mps <= stopped_velocity_threshold_mps_;
        if (!nc_stopped) {
          stable_initialized[idx] = false;
          // An NC velocity job can remain active even when the encoder reports
          // nearly zero speed (e.g. a blocked wheel).  Request ONE guarded STOP
          // for either genuine motion or an outstanding NC job.  The shared
          // request_axis_stop() gate rejects duplicates and refuses PLC STOP
          // when its FB is Busy/Error; it does NOT clear an unresolved job.
          if (!s.bError && std::isfinite(vel_mps) &&
              (vel_mps > stop_suppress_velocity_mps_ || s.bHasJob)) {
            const auto stop_result = request_axis_stop(
              axis, axis == rail_axis_ ? rail_ramp_ms_ : wheel_ramp_ms_,
              "bringup shutdown");
            if (!stop_logged[idx] &&
                stop_result != StopRequestResult::ACCEPTED &&
                stop_result != StopRequestResult::ALREADY_STOPPED) {
              stop_logged[idx] = true;
              RCLCPP_ERROR(this->get_logger(),
                "[SHUTDOWN STOP UNRESOLVED] Axis%d STOP result=%d "
                "Ready=%d NM=%d Job=%d FW=%d BW=%d speed=%.5f m/s. "
                "No duplicate STOP or unsafe PowerDisable will be sent.",
                axis, static_cast<int>(stop_result),
                s.bControlLoopClosed ? 1 : 0, s.bNotMoving ? 1 : 0,
                s.bHasJob ? 1 : 0, s.bMovingFw ? 1 : 0,
                s.bMovingBw ? 1 : 0, vel_mps);
            }
          } else if (!stop_logged[idx]) {
            stop_logged[idx] = true;
            RCLCPP_ERROR(this->get_logger(),
              "[SHUTDOWN WAIT] Axis%d Power ON, NC not idle: "
              "Error=%d NotMoving=%d HasJob=%d FW=%d BW=%d speed=%.5f m/s. "
              "Keeping Power ON until real standstill (no bypass).",
              axis, s.bError ? 1 : 0, s.bNotMoving ? 1 : 0,
              s.bHasJob ? 1 : 0, s.bMovingFw ? 1 : 0,
              s.bMovingBw ? 1 : 0, vel_mps);
          }
          continue;
        }
        FunctionStatus stop{};
        if (!read_stop_status(axis, stop) || stop.bBusy || stop.bError) {
          stable_initialized[idx] = false;
          continue;
        }
        // Any STOP sent in this run must get its ramp+ADS settle window.
        if (last_stop_tx_initialized_[idx] &&
            std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - last_stop_tx_tp_[idx]).count() <
                (axis == rail_axis_ ? rail_ramp_ms_ : wheel_ramp_ms_) +
                ads_command_settle_extra_ms_) {
          stable_initialized[idx] = false;
          continue;
        }
        // Require >=200ms continuous NC idle + Stop_FB not busy.
        const auto now = std::chrono::steady_clock::now();
        if (!stable_initialized[idx]) {
          stable_initialized[idx] = true;
          stable_since[idx] = now;
          continue;
        }
        if (std::chrono::duration<double, std::milli>(
              now - stable_since[idx]).count() < 200.0) continue;

        RCLCPP_WARN(this->get_logger(),
          "[SHUTDOWN POWER OFF TX] Axis%d NC idle verified; sending PowerDisable", axis);
        // Mark attempted BEFORE synchronous RPC; never spam PowerDisable.
        disable_sent[idx] = true;
        if (!power_disable_axis(axis)) {
          RCLCPP_ERROR(this->get_logger(),
            "[SHUTDOWN POWER OFF FAILED] Axis%d ADS command rejected; "
            "do not assume drive disabled", axis);
        }
      }
      if (all_verified) break;
      std::this_thread::sleep_for(100ms);
    }
    // Final fresh feedback; independent of ACK return codes.
    axis_snapshot_valid_ = false;
    std::array<AxisStatus, 3> final_state{};
    if (read_axis_status(final_state)) {
      for (const int axis : axes) {
        if (axis < 1 || axis > 3) continue;
        const auto & s = final_state[axis_index(axis)];
        if (!s.bControlLoopClosed) {
          if (!verified_off[static_cast<std::size_t>(axis)]) {
            RCLCPP_INFO(this->get_logger(),
              "[SHUTDOWN OFF CONFIRMED] Axis%d Ready=0 (final snapshot)", axis);
          }
        } else {
          RCLCPP_ERROR(this->get_logger(),
            "[SHUTDOWN INCOMPLETE] Axis%d STILL POWER ON: Ready=1 "
            "NM=%d Job=%d FW=%d BW=%d Error=%d Actual=%.5f m/s. "
            "PLC safety stop / operator intervention REQUIRED.",
            axis, s.bNotMoving ? 1 : 0, s.bHasJob ? 1 : 0,
            s.bMovingFw ? 1 : 0, s.bMovingBw ? 1 : 0,
            s.bError ? 1 : 0, s.lActVelocity / velocity_scale_);
        }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "[SHUTDOWN UNVERIFIED] Final ADS feedback unavailable. "
        "Do not assume any motor is OFF.");
    }
  }

  bool wait_until_all_axes_stopped(
    double timeout_sec,
    const std::string & reason)
  {
    if (fault_latched_) {
      // The faulted drive must not see StopMove(), even during ROS shutdown.
      std::array<AxisStatus, 3> fault_status{};
      return read_axis_status(fault_status) &&
        !has_any_fault(fault_status) && all_axes_physically_stopped(fault_status);
    }
    clear_motion_requests();
    // Keep the per-axis STOP one-shot gate even across blocking waits.

    const auto start = std::chrono::steady_clock::now();

    while (true) {
      std::array<AxisStatus, 3> status{};

      if (read_axis_status(status)) {
        const auto & left =
          status[axis_index(left_wheel_axis_)];
        const auto & right =
          status[axis_index(right_wheel_axis_)];
        const auto & rail =
          status[axis_index(rail_axis_)];

        const bool left_stopped =
          axis_is_physically_stopped(left, left_wheel_sign_);
        const bool right_stopped =
          axis_is_physically_stopped(right, right_wheel_sign_);
        const bool rail_stopped =
          axis_is_physically_stopped(rail, rail_sign_);

        if (left_stopped && right_stopped && rail_stopped &&
            all_previous_stops_settled() && all_previous_moves_settled()) {
          applied_left_mps_ = 0.0;
          applied_right_mps_ = 0.0;
          applied_rail_mps_ = 0.0;

          RCLCPP_INFO(
            this->get_logger(),
            "[SAFE STOP] All axes physically stopped (%s).",
            reason.c_str());
          return true;
        }

        if (!left_stopped) {
          (void)request_axis_stop(
            left_wheel_axis_, wheel_ramp_ms_, reason.c_str());
        }
        if (!right_stopped) {
          (void)request_axis_stop(
            right_wheel_axis_, wheel_ramp_ms_, reason.c_str());
        }
        if (!rail_stopped) {
          (void)request_axis_stop(
            rail_axis_, rail_ramp_ms_, reason.c_str());
        }
      }

      const double elapsed_sec =
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();

      if (elapsed_sec >= timeout_sec) {
        std::array<AxisStatus, 3> final_status{};
        if (read_axis_status(final_status)) {
          const auto & left =
            final_status[axis_index(left_wheel_axis_)];
          const auto & right =
            final_status[axis_index(right_wheel_axis_)];
          const auto & rail =
            final_status[axis_index(rail_axis_)];

          RCLCPP_ERROR(
            this->get_logger(),
            "[SAFE STOP] Timeout during %s. "
            "L[nm=%d vel=%+.3f] R[nm=%d vel=%+.3f] Rail[nm=%d vel=%+.3f]. "
            "PowerDisable will NOT be executed while an axis is moving.",
            reason.c_str(),
            left.bNotMoving ? 1 : 0,
            (left.lActVelocity / velocity_scale_) * left_wheel_sign_,
            right.bNotMoving ? 1 : 0,
            (right.lActVelocity / velocity_scale_) * right_wheel_sign_,
            rail.bNotMoving ? 1 : 0,
            (rail.lActVelocity / velocity_scale_) * rail_sign_);
        }
        return false;
      }

      std::this_thread::sleep_for(100ms);
    }
  }

  bool wait_until_faults_clear(double timeout_sec)
  {
    const auto start = std::chrono::steady_clock::now();

    while (true) {
      std::array<AxisStatus, 3> status{};
      if (read_axis_status(status) && !has_any_fault(status)) {
        return true;
      }

      const double elapsed_sec =
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();

      if (elapsed_sec >= timeout_sec) {
        return false;
      }

      std::this_thread::sleep_for(100ms);
    }
  }

  // ============================================================
  // Beckhoff initialization
  // ============================================================
  bool rail_mode_requested() const
  {
    return
      rail_state_ == "ON_RAIL" ||
      rail_state_ == "WORKING" ||
      rail_state_ == "EXITING";
  }

  bool wheel_mode_requested() const
  {
    return !rail_mode_requested();
  }

  bool power_enable_axis(int axis)
  {
    if (diagnostic_read_only_) return false;
    const int32_t result = PowerEnable(static_cast<uint32_t>(axis));
    if (result != NO_ERR_TcMCFUN) {
      RCLCPP_ERROR(
        this->get_logger(),
        "PowerEnable(Axis%d) failed: 0x%X",
        axis,
        static_cast<unsigned int>(result));
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "PowerEnable(Axis%d) OK", axis);
    return true;
  }

  bool power_disable_axis(int axis)
  {
    if (diagnostic_read_only_) return false;
    const int32_t result = PowerDisable(static_cast<uint32_t>(axis));
    if (result != NO_ERR_TcMCFUN) {
      RCLCPP_ERROR(
        this->get_logger(),
        "PowerDisable(Axis%d) failed: 0x%X",
        axis,
        static_cast<unsigned int>(result));
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "PowerDisable(Axis%d) OK", axis);
    return true;
  }

  void power_disable_all_axes()
  {
    if (diagnostic_read_only_) return;
    for (const int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const int32_t result = PowerDisable(static_cast<uint32_t>(axis));
      if (result != NO_ERR_TcMCFUN) {
        RCLCPP_WARN(
          this->get_logger(),
          "PowerDisable(Axis%d) failed: 0x%X",
          axis,
          static_cast<unsigned int>(result));
      }
    }
  }

  bool wait_for_expected_power_state(
    bool wheels_enabled,
    bool rail_enabled,
    double timeout_sec)
  {
    const auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
      axis_snapshot_valid_ = false; // Do not confirm a power transition from cached ADS status.
      std::array<AxisStatus, 3> status{};
      if (read_axis_status(status)) {
        const bool left_ready =
          status[axis_index(left_wheel_axis_)].bControlLoopClosed;
        const bool right_ready =
          status[axis_index(right_wheel_axis_)].bControlLoopClosed;
        const bool rail_ready =
          status[axis_index(rail_axis_)].bControlLoopClosed;

        if (
          left_ready == wheels_enabled &&
          right_ready == wheels_enabled &&
          rail_ready == rail_enabled)
        {
          wheel_axes_enabled_ = wheels_enabled;
          rail_axis_enabled_ = rail_enabled;
          return true;
        }
      }

      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= timeout_sec) {
        return false;
      }

      rclcpp::sleep_for(100ms);
    }

    return false;
  }

  bool wait_for_axis_power_off(int axis, double timeout_sec)
  {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      axis_snapshot_valid_ = false;
      std::array<AxisStatus, 3> state{};
      if (read_axis_status(state) && !state[axis_index(axis)].bControlLoopClosed) return true;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= timeout_sec)
        return false;
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  // Read-only startup diagnosis: distinguish mismatched power topology
  // from an Axis1 NC job/MC_Stop lock. This does not send any PLC commands.
  void log_startup_axis_diagnostics(const std::array<AxisStatus, 3> & status)
  {
    const bool target_wheels = wheel_mode_requested();
    const bool target_rail = rail_mode_requested();
    const bool actual_left = status[axis_index(left_wheel_axis_)].bControlLoopClosed;
    const bool actual_right = status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    const bool actual_rail = status[axis_index(rail_axis_)].bControlLoopClosed;
    RCLCPP_DEBUG(this->get_logger(),
      "[STARTUP DIAG POWER] desired Wheel=%d Rail=%d | "
      "PLC Axis%d=%d Axis%d=%d Axis%d=%d | dual_opt_in=%d",
      target_wheels ? 1 : 0, target_rail ? 1 : 0,
      left_wheel_axis_, actual_left ? 1 : 0,
      right_wheel_axis_, actual_right ? 1 : 0,
      rail_axis_, actual_rail ? 1 : 0,
      allow_simultaneous_drive_ ? 1 : 0);
    for (const int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & a = status[axis_index(axis)];
      const double act = a.lActVelocity / velocity_scale_;
      RCLCPP_DEBUG(this->get_logger(),
        "[STARTUP DIAG AXIS%d] Ready=%d NM=%d Job=%d FW=%d BW=%d "
        "Error=%d(0x%X) Actual=%.6f m/s",
        axis, a.bControlLoopClosed ? 1 : 0,
        a.bNotMoving ? 1 : 0, a.bHasJob ? 1 : 0,
        a.bMovingFw ? 1 : 0, a.bMovingBw ? 1 : 0,
        a.bError ? 1 : 0, static_cast<unsigned int>(a.nErrorCode), act);
      if (!a.bControlLoopClosed) continue;
      FunctionStatus stop{}, move{};
      const bool stop_ok = read_stop_status(axis, stop);
      const bool move_ok = read_move_status(axis, move);
      if (!stop_ok || !move_ok) {
        RCLCPP_WARN(this->get_logger(),
          "[STARTUP DIAG FB%d] Unable to read PLC FB status "
          "StopRead=%d MoveRead=%d; no motion authorization",
          axis, stop_ok ? 1 : 0, move_ok ? 1 : 0);
        continue;
      }
      RCLCPP_DEBUG(this->get_logger(),
        "[STARTUP DIAG FB%d] Stop(Busy=%d Done=%d Error=%d Code=0x%X) "
        "Move(Busy=%d Done=%d Error=%d Code=0x%X)",
        axis, stop.bBusy ? 1 : 0, stop.bDone ? 1 : 0,
        stop.bError ? 1 : 0, static_cast<unsigned int>(stop.nError),
        move.bBusy ? 1 : 0, move.bDone ? 1 : 0,
        move.bError ? 1 : 0, static_cast<unsigned int>(move.nError));
    }
  }

  bool configure_power_for_current_state_blocking(double timeout_sec)
  {
    std::array<AxisStatus, 3> status{};
    if (!read_axis_status(status) || has_any_fault(status)) {
      RCLCPP_ERROR(this->get_logger(), "[STARTUP] Fresh AxisStatus unavailable or faulted.");
      return false;
    }
    const bool left_ready = status[axis_index(left_wheel_axis_)].bControlLoopClosed;
    const bool right_ready = status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    const bool rail_ready = status[axis_index(rail_axis_)].bControlLoopClosed;
    log_startup_axis_diagnostics(status);
    // Dual mode NEVER powers on both drive groups automatically at startup.
    // Start with legacy wheel/rail power selection. The operator must enable
    // the second group explicitly while proven stopped.
    const bool expected_wheels = wheel_mode_requested();
    const bool expected_rail = rail_mode_requested();

    // Experimental dual mode must NEVER power-cycle a valid already-enabled
    // combination at startup. Preserve whichever group(s) the PLC currently
    // reports enabled; never turn both on automatically at node creation.
    if (allow_simultaneous_drive_) {
      if (left_ready != right_ready || (!left_ready && !rail_ready) ||
          !plc_idle_for_existing_power(status)) {
        RCLCPP_ERROR(this->get_logger(),
          "[DUAL STARTUP BLOCKED] power mismatch, no enabled drive, or PLC "
          "standstill/FB state not confirmed. No automatic Power/STOP sent.");
        return false;
      }
      wheel_axes_enabled_ = left_ready && right_ready;
      rail_axis_enabled_ = rail_ready;
      RCLCPP_INFO(this->get_logger(),
        "[DUAL STARTUP] Preserved Wheel=%d Rail=%d; no Power/STOP command. "
        "Use E/Y only at confirmed standstill to change enables.",
        wheel_axes_enabled_ ? 1 : 0, rail_axis_enabled_ ? 1 : 0);
      return true;
    }

    // A restart is NOT grounds for STOP/Power cycling. Some PLCs report
    // bNotMoving=0 even with zero velocity and no HasJob; verify both
    // individual PLC FBs are idle before accepting an already-correct mode.
    // This path never authorizes PowerDisable or a mechanical mode switch.
    if (left_ready == expected_wheels && right_ready == expected_wheels &&
        rail_ready == expected_rail) {
      wheel_axes_enabled_ = left_ready && right_ready;
      rail_axis_enabled_ = rail_ready;
      if (!plc_idle_for_existing_power(status)) {
        RCLCPP_ERROR(this->get_logger(),
          "[STARTUP BLOCKED] Power already matches, but the PLC still reports "
          "an active job/Busy/unknown status. No STOP or Power command sent.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
        "[STARTUP] Existing power mode matches %s; no STOP or power cycling. "
        "PLC idle confirmed.", rail_state_.c_str());
      return true;
    }

    // Changing power requires stricter standstill evidence (bNotMoving +
    // near-zero velocity), never inferred from velocity alone.
    RCLCPP_INFO(this->get_logger(),
      "[STARTUP POWER MISMATCH] Expected wheels=%d rail=%d, but actual "
      "Axis%d=%d Axis%d=%d Axis%d=%d. Waiting up to %.1fs for genuine "
      "NC standstill before any Power transition. Check Axis1 HasJob/STOP FB; "
      "zero measured speed does NOT mean bNotMoving=1.",
      expected_wheels ? 1 : 0, expected_rail ? 1 : 0,
      left_wheel_axis_, left_ready ? 1 : 0,
      right_wheel_axis_, right_ready ? 1 : 0,
      rail_axis_, rail_ready ? 1 : 0, timeout_sec);
    if (!wait_until_all_axes_stopped(timeout_sec, "power-mode handover")) {
      RCLCPP_ERROR(this->get_logger(),
        "[STARTUP BLOCKED] Power change requires confirmed standstill; "
        "no unsafe PowerDisable/Enable performed.");
      return false;
    }

    // Disable only axes that actually need it. Do not resend identical Power
    // commands to MC_Power in an otherwise healthy PLC.
    if (left_ready && !expected_wheels && !power_disable_axis(left_wheel_axis_)) return false;
    if (right_ready && !expected_wheels && !power_disable_axis(right_wheel_axis_)) return false;
    if (rail_ready && !expected_rail && !power_disable_axis(rail_axis_)) return false;
    // Wait for each PowerDisable result before enabling an alternative drive.
    const auto disable_start = std::chrono::steady_clock::now();
    while (true) {
      std::array<AxisStatus, 3> chk{};
      if (read_axis_status(chk) &&
          (expected_wheels ||
            (!chk[axis_index(left_wheel_axis_)].bControlLoopClosed &&
             !chk[axis_index(right_wheel_axis_)].bControlLoopClosed)) &&
          (expected_rail || !chk[axis_index(rail_axis_)].bControlLoopClosed)) break;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
          disable_start).count() > timeout_sec) return false;
      std::this_thread::sleep_for(100ms);
    }
    if (expected_rail && !rail_ready && !power_enable_axis(rail_axis_)) return false;
    if (expected_wheels) {
      if (!left_ready && !power_enable_axis(left_wheel_axis_)) return false;
      if (!right_ready && !power_enable_axis(right_wheel_axis_)) return false;
    }
    if (!wait_for_expected_power_state(expected_wheels, expected_rail, timeout_sec)) {
      RCLCPP_ERROR(this->get_logger(), "[STARTUP BLOCKED] Power feedback mismatch.");
      return false;
    }
    return true;
  }

  // Fault recovery must never issue StopMove(). After Reset clears the error,
  // re-arm is allowed ONLY if all axes already report a physical standstill.
  // Uses existing vendor PowerEnable/PowerDisable API; no motion command.
  bool configure_power_after_fault_reset_without_stop(double timeout_sec)
  {
    std::array<AxisStatus, 3> status{};
    if (!read_axis_status(status) || has_any_fault(status)) {
      RCLCPP_ERROR(this->get_logger(),
        "[FAULT RESET] Axis state unavailable or a PLC fault remains; no STOP.");
      return false;
    }

    const bool wheels_ready =
      status[axis_index(left_wheel_axis_)].bControlLoopClosed &&
      status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    const bool rail_ready =
      status[axis_index(rail_axis_)].bControlLoopClosed;

    // Dual mode preserves any already-verified enabled combination; Reset
    // must never silently disable the other drive group.
    if (allow_simultaneous_drive_) {
      const bool left_on =
        status[axis_index(left_wheel_axis_)].bControlLoopClosed;
      const bool right_on =
        status[axis_index(right_wheel_axis_)].bControlLoopClosed;
      if (left_on != right_on || (!wheels_ready && !rail_ready) ||
          !plc_idle_for_existing_power(status)) {
        return false;
      }
      wheel_axes_enabled_ = wheels_ready;
      rail_axis_enabled_ = rail_ready;
      return true;
    }

    // Avoid unnecessary power cycling if the physical mode is already correct.
    if (wheels_ready == wheel_mode_requested() &&
        rail_ready == rail_mode_requested()) {
      // Re-arm an unchanged Power state ONLY when the PLC has no job and
      // both relevant FBs are idle.  Do not rely on bNotMoving alone.
      if (!plc_idle_for_existing_power(status)) return false;
      wheel_axes_enabled_ = wheels_ready;
      rail_axis_enabled_ = rail_ready;
      return true;
    }

    // Changing the physical drive enable state demands strict standstill.
    if (!all_axes_physically_stopped(status)) {
      RCLCPP_ERROR(this->get_logger(),
        "[FAULT RESET] Power change blocked: bNotMoving/velocity inconclusive.");
      return false;
    }

    power_disable_all_axes();
    if (rail_mode_requested()) {
      if (!power_enable_axis(rail_axis_)) {
        return false;
      }
      return wait_for_expected_power_state(false, true, timeout_sec);
    }

    if (!power_enable_axis(left_wheel_axis_) ||
        !power_enable_axis(right_wheel_axis_)) {
      return false;
    }
    return wait_for_expected_power_state(true, false, timeout_sec);
  }

  void connect_and_initialize()
  {
    const int32_t connect_result = Connect();

    if (connect_result != NO_ERR_TcMCFUN) {
      ads_connected_ = false;
      RCLCPP_ERROR(
        this->get_logger(),
        "Connect() failed: 0x%X",
        static_cast<unsigned int>(connect_result));
      return;
    }

    ads_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to Beckhoff TwinCAT ADS.");
    // Share the vendor API endpoint instead of independently selecting one.
    const AmsNetId plc_net_id REMOTE_NET_ID;
    if (!plc_fb_reader_.open(plc_net_id)) {
      RCLCPP_ERROR(this->get_logger(),
        "[PLC STATUS] Could not open read-only Stop/Move status handles; motion blocked.");
      motion_ready_ = false;
      return;
    }

    // Vendor-provided ReadAxisStatus() performs a synchronous snapshot read.
    // Avoid their asynchronous memcpy Notification cache's data race without
    // editing any vendor file.  Poll at most once per 50 ms.

    // Wait for a real first ADS snapshot, not zero-initialized axis data.
    // Do not start the motors if the PLC state cannot be read.
    {
      const auto begin = std::chrono::steady_clock::now();
      std::array<AxisStatus, 3> first{};
      while (!read_axis_status(first)) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
              .count() >= 3.0) {
          RCLCPP_ERROR(this->get_logger(),
            "[STARTUP] No readable ADS AxisStatus within 3 seconds. Motion blocked.");
          motion_ready_ = false;
          return;
        }
        std::this_thread::sleep_for(100ms);
      }
    }

    start_environment_sensor_polling();

    std::array<AxisStatus, 3> status{};
    if (read_axis_status(status) && has_any_fault(status)) {
      motion_ready_ = false;
      fault_latched_ = true;
      RCLCPP_ERROR(
        this->get_logger(),
        "Axis fault exists at startup. Remove the cause, call /agv/reset_faults, "
        "then motion can resume.");
      return;
    }

    if (diagnostic_read_only_ || !power_enable_on_start_) {
      motion_ready_ = false;
      RCLCPP_WARN(this->get_logger(),
        "[STARTUP DIAGNOSTIC ONLY] diagnostic_read_only=%d, "
        "power_enable_on_start=%d. ADS motor commands are blocked "
        "only when diagnostic_read_only=1; existing PLC motor enables remain unchanged.",
        diagnostic_read_only_ ? 1 : 0,
        power_enable_on_start_ ? 1 : 0);
      // Read-only FB/NC snapshot; do not attempt to recover or unlock MC_Stop.
      if (read_axis_status(status)) {
        log_startup_axis_diagnostics(status);
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "[STARTUP DIAGNOSTIC ONLY] AxisStatus read failed.");
      }
      return;
    }

    // Startup state defaults to OUT_RAIL:
    //   Wheel Axis1/2 ENABLE, Rail Axis3 DISABLE.
    // This directly enforces the vendor requirement that the rail motor is
    // disabled while the AGV approaches and climbs onto the rail.
    if (!configure_power_for_current_state_blocking(startup_power_transition_timeout_sec_)) {
      motion_ready_ = false;
      std::array<AxisStatus, 3> blocked{};
      if (read_axis_status(blocked)) {
        RCLCPP_ERROR(this->get_logger(),
          "[STARTUP DIAG] Re-read axis and FB states after startup failure:");
        log_startup_axis_diagnostics(blocked);
      }
      return;
    }

    motion_ready_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Initial power policy applied for rail_state=%s (Wheels=%s, Rail=%s)",
      rail_state_.c_str(),
      wheel_axes_enabled_ ? "ENABLE" : "DISABLE",
      rail_axis_enabled_ ? "ENABLE" : "DISABLE");
  }

  enum class PowerTransitionStage
  {
    NONE,
    STOP_WHEELS,
    DISABLE_WHEELS,
    ENABLE_RAIL,
    STOP_RAIL,
    DISABLE_RAIL,
    ENABLE_WHEELS
  };

  void begin_power_transition_for_state()
  {
    // Do not reset the STOP one-shot gate here; the last command may still
    // be executing. A successfully accepted MoveVelocity re-arms it.

    // Always clear stale motion when the mechanical drive mode changes.
    requested_linear_x_mps_ = 0.0;
    requested_angular_z_rps_ = 0.0;
    requested_rail_speed_mps_ = 0.0;

    left_pending_target_mps_ = 0.0;
    right_pending_target_mps_ = 0.0;
    rail_pending_target_mps_ = 0.0;
    left_waiting_for_stop_ = false;
    right_waiting_for_stop_ = false;
    rail_waiting_for_stop_ = false;
    rail_stop_hold_active_ = false;

    if (mock_mode_) {
      wheel_axes_enabled_ = !rail_mode_requested();
      rail_axis_enabled_ = rail_mode_requested();
      power_transition_stage_ = PowerTransitionStage::NONE;
      power_stage_command_sent_ = false;
      return;
    }

    power_stage_command_sent_ = false;

    if (rail_mode_requested()) {
      power_transition_stage_ = PowerTransitionStage::STOP_WHEELS;
    } else {
      power_transition_stage_ = PowerTransitionStage::STOP_RAIL;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "[POWER INTERLOCK] rail_state=%s -> target Wheels=%s / Rail=%s",
      rail_state_.c_str(),
      rail_mode_requested() ? "DISABLE" : "ENABLE",
      rail_mode_requested() ? "ENABLE" : "DISABLE");
  }

  void process_power_transition(const std::array<AxisStatus, 3> & status)
  {
    const bool left_ready =
      status[axis_index(left_wheel_axis_)].bControlLoopClosed;
    const bool right_ready =
      status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    const bool rail_ready =
      status[axis_index(rail_axis_)].bControlLoopClosed;

    wheel_axes_enabled_ = left_ready && right_ready;
    rail_axis_enabled_ = rail_ready;

    switch (power_transition_stage_) {
      case PowerTransitionStage::NONE:
        return;

      case PowerTransitionStage::STOP_WHEELS:
      {
        const auto & left_status =
          status[axis_index(left_wheel_axis_)];
        const auto & right_status =
          status[axis_index(right_wheel_axis_)];

        const bool left_stopped =
          axis_is_physically_stopped(left_status, left_wheel_sign_);
        const bool right_stopped =
          axis_is_physically_stopped(right_status, right_wheel_sign_);

        // Even if /cmd_vel is still published by Nav2, this transition owns
        // the wheels. BUSY is retried with backoff; no PowerDisable yet.
        if (!left_stopped) {
          const auto r =
            request_axis_stop(
              left_wheel_axis_, wheel_ramp_ms_, "wheel->rail power transition");
          if (r == StopRequestResult::ACCEPTED) {
            applied_left_mps_ = 0.0;
          }
        }

        if (!right_stopped) {
          const auto r =
            request_axis_stop(
              right_wheel_axis_, wheel_ramp_ms_, "wheel->rail power transition");
          if (r == StopRequestResult::ACCEPTED) {
            applied_right_mps_ = 0.0;
          }
        }

        if (left_stopped && right_stopped &&
            all_previous_stops_settled() && all_previous_moves_settled()) {
          power_transition_stage_ = PowerTransitionStage::DISABLE_WHEELS;
          power_stage_command_sent_ = false;
          RCLCPP_INFO(
            this->get_logger(),
            "[POWER INTERLOCK] Wheels physically stopped; PowerDisable may proceed.");
        }
        return;
      }

      case PowerTransitionStage::DISABLE_WHEELS:
        if (!left_ready && !right_ready) {
          wheel_axes_enabled_ = false;
          power_transition_stage_ = PowerTransitionStage::ENABLE_RAIL;
          power_stage_command_sent_ = false;
          return;
        }

        if (!power_stage_command_sent_) {
          bool ok = true;
          if (left_ready) {
            ok = power_disable_axis(left_wheel_axis_) && ok;
          }
          if (right_ready) {
            ok = power_disable_axis(right_wheel_axis_) && ok;
          }
          power_stage_command_sent_ = ok;
        }
        return;

      case PowerTransitionStage::ENABLE_RAIL:
        if (rail_ready) {
          rail_axis_enabled_ = true;
          power_transition_stage_ = PowerTransitionStage::NONE;
          power_stage_command_sent_ = false;
          RCLCPP_INFO(
            this->get_logger(),
            "[POWER INTERLOCK] Rail mode ready: Wheels=DISABLE, Rail=ENABLE");
          return;
        }

        if (!power_stage_command_sent_) {
          power_stage_command_sent_ = power_enable_axis(rail_axis_);
        }
        return;

      case PowerTransitionStage::STOP_RAIL:
      {
        const auto & rail_status =
          status[axis_index(rail_axis_)];

        const bool rail_stopped =
          axis_is_physically_stopped(rail_status, rail_sign_);

        rail_pending_target_mps_ = 0.0;
        rail_waiting_for_stop_ = false;
        rail_stop_hold_active_ = false;

        if (!rail_stopped) {
          const auto r =
            request_axis_stop(
              rail_axis_, rail_ramp_ms_, "rail->wheel power transition");
          if (r == StopRequestResult::ACCEPTED) {
            applied_rail_mps_ = 0.0;
          }
        }

        if (rail_stopped &&
            all_previous_stops_settled() && all_previous_moves_settled()) {
          power_transition_stage_ = PowerTransitionStage::DISABLE_RAIL;
          power_stage_command_sent_ = false;
          RCLCPP_INFO(
            this->get_logger(),
            "[POWER INTERLOCK] Rail physically stopped; PowerDisable may proceed.");
        }
        return;
      }

      case PowerTransitionStage::DISABLE_RAIL:
        if (!rail_ready) {
          rail_axis_enabled_ = false;
          power_transition_stage_ = PowerTransitionStage::ENABLE_WHEELS;
          power_stage_command_sent_ = false;
          return;
        }

        if (!power_stage_command_sent_) {
          power_stage_command_sent_ = power_disable_axis(rail_axis_);
        }
        return;

      case PowerTransitionStage::ENABLE_WHEELS:
        if (left_ready && right_ready) {
          wheel_axes_enabled_ = true;
          power_transition_stage_ = PowerTransitionStage::NONE;
          power_stage_command_sent_ = false;
          RCLCPP_INFO(
            this->get_logger(),
            "[POWER INTERLOCK] Ground mode ready: Wheels=ENABLE, Rail=DISABLE");
          return;
        }

        if (!power_stage_command_sent_) {
          bool ok = true;
          if (!left_ready) {
            ok = power_enable_axis(left_wheel_axis_) && ok;
          }
          if (!right_ready) {
            ok = power_enable_axis(right_wheel_axis_) && ok;
          }
          power_stage_command_sent_ = ok;
        }
        return;
    }
  }

  // ============================================================
  // ROS callbacks
  // ============================================================
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (diagnostic_read_only_) {
      requested_linear_x_mps_ = 0.0;
      requested_angular_z_rps_ = 0.0;
      wheel_motion_armed_ = false;
      return;
    }
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      RCLCPP_WARN(this->get_logger(), "Invalid /cmd_vel received.");
      return;
    }

    last_cmd_vel_time_ = this->now();
    last_cmd_vel_steady_ = std::chrono::steady_clock::now();

    if (safe_stop_latched_ || fault_latched_) {
      requested_linear_x_mps_ = 0.0;
      requested_angular_z_rps_ = 0.0;
      // The transition into SAFE STOP is already logged once.
      // Do not spam heartbeat rejection messages.
      return;
    }

    wheel_motion_armed_ =
      (std::fabs(msg->linear.x) > 1.0e-6 || std::fabs(msg->angular.z) > 1.0e-6);

    // Wheels must not receive a stored non-zero command while the robot is
    // ON_RAIL/WORKING/EXITING. A fresh /cmd_vel is required after OUT_RAIL.
    if ((!allow_simultaneous_drive_ && !wheel_mode_requested()) ||
        power_transition_stage_ != PowerTransitionStage::NONE ||
        !wheel_axes_enabled_)
    {
      requested_linear_x_mps_ = 0.0;
      requested_angular_z_rps_ = 0.0;
      return;
    }

    requested_linear_x_mps_ = msg->linear.x;
    requested_angular_z_rps_ = msg->angular.z;
  }

  void rail_cmd_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (diagnostic_read_only_) {
      requested_rail_speed_mps_ = 0.0;
      rail_motion_armed_ = false;
      return;
    }
    const double speed_mps = static_cast<double>(msg->data);

    if (!std::isfinite(speed_mps)) {
      RCLCPP_WARN(this->get_logger(), "Invalid /rail/cmd_speed received.");
      return;
    }

    last_rail_cmd_time_ = this->now();
    last_rail_cmd_steady_ = std::chrono::steady_clock::now();

    if (safe_stop_latched_ || fault_latched_) {
      requested_rail_speed_mps_ = 0.0;
      // Expected heartbeat while latched; omit repetitive warning.
      return;
    }

    rail_motion_armed_ = (std::fabs(speed_mps) > 1.0e-6);

    // Rail Axis3 is physically disabled before ON_RAIL. Never retain an old
    // non-zero rail command that could execute after PowerEnable.
    if ((!allow_simultaneous_drive_ && !rail_mode_requested()) ||
        power_transition_stage_ != PowerTransitionStage::NONE ||
        !rail_axis_enabled_)
    {
      requested_rail_speed_mps_ = 0.0;
      return;
    }

    requested_rail_speed_mps_ = speed_mps;
  }

  void rail_state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (diagnostic_read_only_) {
      // Ignore /rail_state changes for power switching; no ADS writes.
      return;
    }
    const std::string new_state = msg->data;

    if (
      new_state != "OUT_RAIL" &&
      new_state != "MOVING_TO_RAIL" &&
      new_state != "RAIL_APPROACH" &&
      new_state != "ENTERING" &&
      new_state != "ON_RAIL" &&
      new_state != "WORKING" &&
      new_state != "EXITING")
    {
      RCLCPP_WARN(this->get_logger(), "Unknown /rail_state: %s", new_state.c_str());
      return;
    }

    if (new_state == rail_state_) {
      return;
    }

    // Never restart/overwrite a power handover halfway through. The operator
    // may request the next state only after the previous transition completes.
    if (!allow_simultaneous_drive_ &&
        power_transition_stage_ != PowerTransitionStage::NONE) {
      RCLCPP_WARN(this->get_logger(),
        "[POWER INTERLOCK] Rejecting %s while transition to %s is in progress",
        new_state.c_str(), rail_state_.c_str());
      return;
    }

    const bool previous_rail_mode = rail_mode_requested();
    const std::string previous_state = rail_state_;
    rail_state_ = new_state;
    const bool new_rail_mode = rail_mode_requested();

    RCLCPP_INFO(
      this->get_logger(),
      "Rail state: %s -> %s",
      previous_state.c_str(),
      rail_state_.c_str());

    // In opted-in dual mode, rail state is informational. NEVER automatically
    // disable Wheel or Rail power when /rail_state changes; revoke any stale
    // motion targets and require fresh keyboard commands after changing state.
    if (allow_simultaneous_drive_) {
      clear_motion_requests();
      wheel_motion_armed_ = false;
      rail_motion_armed_ = false;
      RCLCPP_WARN(this->get_logger(),
        "[DUAL DRIVE] rail_state=%s: keeping both Power states unchanged; "
        "motion targets cleared", rail_state_.c_str());
      return;
    }
    // Only a change between ground-drive mode and rail-drive mode requires
    // a servo power handover. WORKING <-> EXITING remains Rail mode.
    if (previous_rail_mode != new_rail_mode) {
      if (!mock_mode_ && !motion_ready_) {
        RCLCPP_ERROR(this->get_logger(),
          "[POWER INTERLOCK] Mode change received while motion not ready; "
          "power switching BLOCKED. Re-arm/diagnose PLC first.");
        power_transition_stage_ = PowerTransitionStage::NONE;
        rail_state_ = previous_state; // Never commit an unexecuted mode switch.
        clear_motion_requests();
        return;
      }
      begin_power_transition_for_state();
    }
  }

  // Shared preconditions for both manual Power services; no motion/STOP is sent.
  bool check_power_service_preconditions(
    bool enable, const char *group,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> &response)
  {
    if (ads_recovery_required_) {
      response->success = false;
      response->message = "ADS reconnect recovery locked; use /agv/approve_ads_recovery after PLC idle verification.";
      return false;
    }
    if (diagnostic_read_only_) {
      response->success = false;
      response->message = "DIAGNOSTIC READ-ONLY: PLC Power/Reset writes are disabled.";
      return false;
    }
    if (enable && (fault_latched_ || safe_stop_latched_)) {
      response->success = false;
      response->message = std::string(group) + " PowerEnable rejected: fault/safe-stop latch is active. Call /agv/reset_faults first.";
      return false;
    }
    if (mock_mode_) return true; // Mock-specific transitions remain in each callback.
    if (!ads_connected_) {
      response->success = false;
      response->message = "ADS is not connected.";
      return false;
    }
    if (power_transition_stage_ != PowerTransitionStage::NONE ||
        reset_rearm_pending_ || stale_recovery_pending_) {
      response->success = false;
      response->message = std::string(group) + " power denied: transition/reset/recovery in progress.";
      return false;
    }
    return true;
  }

  void wheel_power_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!check_power_service_preconditions(request->data, "Wheel", response)) return;

    if (mock_mode_) {
      if (request->data) {
        wheel_axes_enabled_ = true;
        if (!allow_simultaneous_drive_) rail_axis_enabled_ = false;
        response->message = allow_simultaneous_drive_ ?
          "Mock DUAL: wheel ON, rail unchanged." : "Mock: wheel power ON, rail power OFF.";
      } else {
        wheel_axes_enabled_ = false;
        response->message = "Mock: wheel power OFF.";
      }
      response->success = true;
      return;
    }

    // Manual power services must NEVER send StopMove just to power-toggle.
    // Deny until all axes are confirmed stopped by PLC feedback.
    std::array<AxisStatus, 3> power_status{};
    if (!read_axis_status(power_status) || has_any_fault(power_status)) {
      response->success = false;
      response->message =
        "Wheel power denied: AxisStatus missing/faulted; NO StopMove sent.";
      return;
    }


    // Idempotent power command: skip duplicate PowerEnable/Disable calls.
    const bool wheels_now =
      power_status[axis_index(left_wheel_axis_)].bControlLoopClosed &&
      power_status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    const bool rail_now =
      power_status[axis_index(rail_axis_)].bControlLoopClosed;

    if (allow_simultaneous_drive_) {
      // Do NOT switch the rail OFF just because a wheel was enabled.
      const bool both_wheels_off =
        !power_status[axis_index(left_wheel_axis_)].bControlLoopClosed &&
        !power_status[axis_index(right_wheel_axis_)].bControlLoopClosed;
      if ((request->data && wheels_now) || (!request->data && both_wheels_off)) {
        response->success = true;
        response->message = "DUAL: wheel power already matches; rail untouched.";
        return;
      }
      if (!motion_ready_ ||
          power_transition_stage_ != PowerTransitionStage::NONE ||
          !all_axes_physically_stopped(power_status) ||
          !all_previous_stops_settled() || !all_previous_moves_settled()) {
        response->success = false;
        response->message = "DUAL wheel power denied: requires motion ready, "
          "all axes at confirmed standstill and PLC FB settled. Rail untouched.";
        return;
      }
      clear_motion_requests();
      wheel_motion_armed_ = false;
      rail_motion_armed_ = false;
      bool ok = true;
      if (request->data) {
        if (!power_status[axis_index(left_wheel_axis_)].bControlLoopClosed)
          ok = power_enable_axis(left_wheel_axis_);
        if (ok && !power_status[axis_index(right_wheel_axis_)].bControlLoopClosed)
          ok = power_enable_axis(right_wheel_axis_);
      } else {
        if (power_status[axis_index(left_wheel_axis_)].bControlLoopClosed)
          ok = power_disable_axis(left_wheel_axis_);
        if (ok && power_status[axis_index(right_wheel_axis_)].bControlLoopClosed)
          ok = power_disable_axis(right_wheel_axis_);
      }
      axis_snapshot_valid_ = false;
      const bool match = ok && wait_for_expected_power_state(request->data, rail_now, kPowerFeedbackTimeoutSec);
      response->success = match;
      response->message = match ? "DUAL: wheels changed; rail power unchanged." :
        "DUAL: wheel power failed/feedback mismatch; check PLC state.";
      if (!match) {
        motion_ready_ = false;
        clear_motion_requests();
      }
      return;
    }
    const bool both_wheels_off =
      !power_status[axis_index(left_wheel_axis_)].bControlLoopClosed &&
      !power_status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    if ((request->data && wheels_now && !rail_now) ||
        (!request->data && both_wheels_off)) {
      response->success = true;
      response->message = request->data ? "Wheels already enabled; no ADS command." :
        "Wheels already disabled; no ADS command.";
      return;
    }

    if (!all_axes_physically_stopped(power_status) ||
        !all_previous_stops_settled() || !all_previous_moves_settled()) {
      response->success = false;
      response->message = "Wheel power denied: standstill/FB completion not confirmed. No STOP sent.";
      return;
    }

    clear_motion_requests();
    wheel_motion_armed_ = false;
    rail_motion_armed_ = false;

    bool ok = true;

    if (request->data) {
      // Never enable the wheels until the rail is confirmed DISABLED.
      // Short-circuit on failed PowerDisable, not 'ok = call() && ok'.
      if (rail_now) {
        if (!power_disable_axis(rail_axis_)) {
          response->success = false;
          response->message = "Rail PowerDisable failed; wheels were NOT enabled.";
          return;
        }
        if (!wait_for_axis_power_off(rail_axis_, kPowerFeedbackTimeoutSec)) {
          response->success = false;
          response->message = "Rail PowerDisable feedback timeout; wheels were NOT enabled.";
          motion_ready_ = false;
          return;
        }
      }
      if (!power_status[axis_index(left_wheel_axis_)].bControlLoopClosed) {
        ok = power_enable_axis(left_wheel_axis_);
      }
      if (ok && !power_status[axis_index(right_wheel_axis_)].bControlLoopClosed) {
        ok = power_enable_axis(right_wheel_axis_);
      }
      if (ok && wait_for_expected_power_state(true, false, kPowerFeedbackTimeoutSec)) {
        wheel_axes_enabled_ = true;
        rail_axis_enabled_ = false;
        motion_ready_ = true;
        response->success = true;
        response->message = "Wheel power ON; rail power OFF.";
      } else {
        response->success = false;
        response->message = "Wheel PowerEnable failed or feedback did not match.";
      }
      return;
    }

    if (power_status[axis_index(left_wheel_axis_)].bControlLoopClosed)
      ok = power_disable_axis(left_wheel_axis_) && ok;
    if (power_status[axis_index(right_wheel_axis_)].bControlLoopClosed)
      ok = power_disable_axis(right_wheel_axis_) && ok;
    // A successful ADS write is not proof of actual drive disable.
    const bool off_confirmed = ok &&
      wait_for_expected_power_state(false, rail_now, kPowerFeedbackTimeoutSec);
    response->success = off_confirmed;
    response->message = off_confirmed ? "Wheel power OFF confirmed by PLC feedback." :
      "Wheel disable failed or PLC OFF feedback timeout; physical power state UNKNOWN.";
    if (!off_confirmed) {
      motion_ready_ = false;
      clear_motion_requests();
    }
  }

  void rail_power_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!check_power_service_preconditions(request->data, "Rail", response)) return;

    if (mock_mode_) {
      if (request->data) {
        if (!allow_simultaneous_drive_) wheel_axes_enabled_ = false;
        rail_axis_enabled_ = true;
        response->message = allow_simultaneous_drive_ ?
          "Mock DUAL: rail ON, wheels unchanged." : "Mock: rail power ON, wheel power OFF.";
      } else {
        rail_axis_enabled_ = false;
        response->message = "Mock: rail power OFF.";
      }
      response->success = true;
      return;
    }

    std::array<AxisStatus, 3> power_status{};
    if (!read_axis_status(power_status) || has_any_fault(power_status)) {
      response->success = false;
      response->message =
        "Rail power denied: AxisStatus missing/faulted; NO StopMove sent.";
      return;
    }


    // Idempotent power command: skip duplicate PowerEnable/Disable calls.
    const bool wheels_now =
      power_status[axis_index(left_wheel_axis_)].bControlLoopClosed &&
      power_status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    const bool rail_now =
      power_status[axis_index(rail_axis_)].bControlLoopClosed;

    if (allow_simultaneous_drive_) {
      // Preserve the existing Wheel power. Power changes MUST be made from
      // proven standstill (not just zero speed) with PLC FB status settled.
      if ((request->data && rail_now) || (!request->data && !rail_now)) {
        response->success = true;
        response->message = "DUAL: rail power already matches; wheels untouched.";
        return;
      }
      if (!motion_ready_ ||
          power_transition_stage_ != PowerTransitionStage::NONE ||
          !all_axes_physically_stopped(power_status) ||
          !all_previous_stops_settled() || !all_previous_moves_settled()) {
        response->success = false;
        response->message = "DUAL rail power denied: requires motion ready, "
          "all axes at confirmed standstill and PLC FB settled. Wheels untouched.";
        return;
      }
      clear_motion_requests();
      wheel_motion_armed_ = false;
      rail_motion_armed_ = false;
      const bool ok = request->data ? power_enable_axis(rail_axis_) :
        power_disable_axis(rail_axis_);
      axis_snapshot_valid_ = false;
      const bool match = ok && wait_for_expected_power_state(wheels_now, request->data, kPowerFeedbackTimeoutSec);
      response->success = match;
      response->message = match ? "DUAL: rail power changed; wheels unchanged." :
        "DUAL: rail power failed/feedback mismatch; check PLC state.";
      if (!match) {
        motion_ready_ = false;
        clear_motion_requests();
      }
      return;
    }
    if ((request->data && rail_now && !wheels_now) ||
        (!request->data && !rail_now)) {
      response->success = true;
      response->message = request->data ? "Rail already enabled; no ADS command." :
        "Rail already disabled; no ADS command.";
      return;
    }

    if (!all_axes_physically_stopped(power_status) ||
        !all_previous_stops_settled() || !all_previous_moves_settled()) {
      response->success = false;
      response->message = "Rail power denied: standstill/FB completion not confirmed. No STOP sent.";
      return;
    }

    clear_motion_requests();
    wheel_motion_armed_ = false;
    rail_motion_armed_ = false;

    bool ok = true;

    if (request->data) {
      // Wheel -> rail: disable both active wheels BEFORE issuing rail ON.
      // Never issue rail PowerEnable if either wheel is still controller-ready.
      if (power_status[axis_index(left_wheel_axis_)].bControlLoopClosed) {
        ok = power_disable_axis(left_wheel_axis_);
      }
      if (ok && power_status[axis_index(right_wheel_axis_)].bControlLoopClosed) {
        ok = power_disable_axis(right_wheel_axis_);
      }
      if (!ok) {
        response->success = false;
        response->message = "Wheel PowerDisable failed. Rail was NOT enabled.";
        return;
      }
      if (!wait_for_expected_power_state(false, rail_now, kPowerFeedbackTimeoutSec)) {
        response->success = false;
        response->message = "Wheel PowerDisable feedback timeout; rail was NOT enabled.";
        motion_ready_ = false;
        return;
      }
      if (!rail_now) ok = power_enable_axis(rail_axis_);
      if (ok && wait_for_expected_power_state(false, true, kPowerFeedbackTimeoutSec)) {
        wheel_axes_enabled_ = false;
        rail_axis_enabled_ = true;
        motion_ready_ = true;
        response->success = true;
        response->message = "Rail power ON; wheel power OFF.";
      } else {
        response->success = false;
        response->message = "Rail PowerEnable failed or feedback did not match.";
      }
      return;
    }

    if (power_status[axis_index(rail_axis_)].bControlLoopClosed)
      ok = power_disable_axis(rail_axis_) && ok;
    const bool off_confirmed = ok &&
      wait_for_expected_power_state(wheels_now, false, kPowerFeedbackTimeoutSec);
    response->success = off_confirmed;
    response->message = off_confirmed ? "Rail power OFF confirmed by PLC feedback." :
      "Rail disable failed or PLC OFF feedback timeout; physical power state UNKNOWN.";
    if (!off_confirmed) {
      motion_ready_ = false;
      clear_motion_requests();
    }
  }

  // ------------------------------------------------------------------
  // EXPLICIT operator recovery from a stale NC MoveVelocity job, used when
  // the robot is confirmed stationary, but HasJob=1 and NotMoving=0.
  // Does NOT bypass bNotMoving, release a motor Power interlock, or re-arm.
  // No automatic STOP command is sent at boot.  An external physical E-stop
  // is necessary because software alone is not a safety function.
  // ------------------------------------------------------------------
  void recover_stale_jobs_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (ads_recovery_required_ || diagnostic_read_only_ || mock_mode_ || !ads_connected_ ||
        stale_recovery_pending_ || reset_rearm_pending_ ||
        power_transition_stage_ != PowerTransitionStage::NONE) {
      response->success = false;
      response->message = "Recovery denied: read-only/mock/offline, already pending, or power transition active.";
      return;
    }
    std::array<AxisStatus, 3> axes{};
    if (!read_axis_status(axes) || has_any_fault(axes)) {
      response->success = false;
      response->message = "Recovery denied: missing axis status or a genuine PLC error (use normal fault procedure).";
      return;
    }
    uint32_t stale_mask = 0;
    for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & st = axes[axis_index(axis)];
      const double mps = std::fabs(st.lActVelocity / velocity_scale_);
      if (!std::isfinite(mps) || mps > kStaleRecoveryMaxVelocityMps) {
        response->success = false;
        response->message = "Recovery denied: an axis is not confirmed near zero velocity.";
        return;
      }
      if (st.bHasJob && !st.bNotMoving) {
        FunctionStatus stop{};
        if (!read_stop_status(axis, stop) || stop.bError || stop.bBusy) {
          response->success = false;
          response->message = "Recovery denied: stale axis STOP FB busy/error/unreadable. PLC service required.";
          return;
        }
        stale_mask |= (uint32_t{1} << (axis-1));
      } else if (st.bHasJob || !st.bNotMoving || st.bMovingFw || st.bMovingBw) {
        response->success = false;
        response->message = "Recovery denied: inconsistent NC state outside stale-job pattern.";
        return;
      }
    }
    if (!stale_mask) {
      response->success = false;
      response->message = "No stale motion job. Use normal /agv/reset_faults or check power/rail mode.";
      return;
    }

    // Leave motor Power unchanged and inhibit all motion throughout recovery.
    clear_motion_requests();
    motion_ready_ = false;
    safe_stop_latched_ = true;
    safe_stop_active_ = false;
    if (safe_stop_reason_.empty()) safe_stop_reason_ = "operator stale-job STOP recovery";
    stale_recovery_pending_ = true;
    stale_recovery_stops_sent_ = false;
    stale_recovery_mask_ = stale_mask;
    stale_recovery_started_ = std::chrono::steady_clock::now();
    RCLCPP_WARN(this->get_logger(),
      "[STALE RECOVERY REQUEST] axes mask=0x%X. Zero velocity must persist for 1.0s. "
      "Then issue exactly ONE STOP per stale axis; NO Power/Move/automatic re-arm.",
      static_cast<unsigned int>(stale_mask));
    response->success = true;
    response->message = "Stale-job recovery armed. No motor motion allowed. Watch [STALE RECOVERY] logs.";
  }

  void process_stale_recovery(const std::array<AxisStatus, 3> & axes)
  {
    if (!stale_recovery_pending_) return;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - stale_recovery_started_).count();
    auto abort = [&](const char *why) {
      stale_recovery_pending_ = false;
      RCLCPP_ERROR(this->get_logger(),
        "[STALE RECOVERY FAILED] %s; motion remains latched; no automatic restart.", why);
    };
    if (elapsed >= kStaleRecoveryTimeoutSec) {
      abort("PLC did not clear stale job/Stop FB within 8s");
      return;
    }
    if (has_any_fault(axes) || power_transition_stage_ != PowerTransitionStage::NONE) {
      abort("PLC fault or power transition appeared");
      return;
    }
    for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & st = axes[axis_index(axis)];
      const double speed = std::fabs(st.lActVelocity / velocity_scale_);
      // Conservative: no 'speed near zero means NotMoving' shortcut.
      if (!std::isfinite(speed) || speed > kStaleRecoveryMaxVelocityMps) {
        abort("axis velocity changed; no additional STOP/Reset/Power commands");
        return;
      }
      if (!stale_recovery_stops_sent_ &&
          (stale_recovery_mask_ & (uint32_t{1} << (axis-1)))) {
        FunctionStatus stop{};
        if (!read_stop_status(axis, stop) || stop.bError || stop.bBusy) {
          abort("STOP FB busy/error/unreadable before one-shot STOP");
          return;
        }
      }
    }
    if (!stale_recovery_stops_sent_) {
      if (elapsed < kStaleRecoveryStandstillSec) return; // continuous stationary validation
      stale_recovery_stops_sent_ = true; // set gate BEFORE ADS writes
      for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
        if (!(stale_recovery_mask_ & (uint32_t{1} << (axis-1)))) continue;
        int32_t result = UNKNOWN_ERR_TcMCFUN;
        RCLCPP_WARN(this->get_logger(),
          "[STALE RECOVERY STOP] Sending one PLC STOP (NOT a velocity command) to Axis%d", axis);
        const double ramp = axis == rail_axis_ ? rail_ramp_ms_ : wheel_ramp_ms_;
        if (!plc_fb_reader_.send_stop_once(axis, ramp, jerk_percent_, result)) {
          RCLCPP_ERROR(this->get_logger(),
            "[STALE RECOVERY STOP FAILED] Axis%d ADS/PLC result=0x%X", axis,
            static_cast<unsigned>(result));
          abort("PLC STOP method rejected. No retries");
          return;
        }
      }
      RCLCPP_WARN(this->get_logger(),
        "[STALE RECOVERY STOP SENT] Observing PLC NC/FB; motor Power unchanged.");
      return;
    }
    bool all_idle = true;
    for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & st = axes[axis_index(axis)];
      if (st.bHasJob || !st.bNotMoving || st.bMovingFw || st.bMovingBw) all_idle = false;
      if (st.bControlLoopClosed) {
        FunctionStatus stop{};
        if (!read_stop_status(axis, stop) || stop.bError || (stop.bBusy && !stop.bDone))
          all_idle = false;
      }
    }
    if (all_idle) {
      stale_recovery_pending_ = false;
      RCLCPP_WARN(this->get_logger(),
        "[STALE RECOVERY COMPLETE] NC jobs cleared; NO auto re-arm or Power switching. "
        "If MC_Stop remains Busy=1 Done=1, /agv/reset_faults can unlock it once. "
        "Confirm rail position/power mode, then request /agv/reset_faults. "
        "If power mode mismatches, restart ROS bringup (IPC reboot NOT required)." );
    }
  }

  // Read-only diagnosis; never request STOP/Reset/Power or TwinCAT RUN/STOP.
  // This is available in the latched state where normal teleop is denied.
  void recovery_diagnostics_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::ostringstream out;
    out << "motion_ready=" << motion_ready_
        << " fault_latched=" << fault_latched_
        << " safe_stop_latched=" << safe_stop_latched_
        << " reset_pending=" << reset_rearm_pending_
        << " ads_recovery_required=" << ads_recovery_required_
        << " ads_transport_pending=" << ads_reconnect_transport_needed_
        << " stale_recovery_pending=" << stale_recovery_pending_
        << " rail_state=" << rail_state_;
    if (!safe_stop_reason_.empty()) out << " latch_reason=" << safe_stop_reason_;
    if (mock_mode_) {
      response->success = false;
      response->message = out.str() + " | MOCK MODE";
      return;
    }
    if (!ads_connected_ || ads_reconnect_transport_needed_) {
      response->success = false;
      response->message = out.str() +
        " | ADS unavailable. Wait for reconnect, then inspect PLC and approve ADS recovery.";
      return;
    }
    std::array<AxisStatus, 3> axes{};
    axis_snapshot_valid_ = false;
    if (!read_axis_status(axes)) {
      response->success = false;
      response->message = out.str() +
        " | AxisStatus unreadable: NO restart or motion authorization from telemetry.";
      return;
    }
    bool any_error = false;
    bool stale_job = false;
    bool stop_fb_blocked = false;
    bool axis_motion_unresolved = false;
    bool power_on = false;
    for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & a = axes[axis_index(axis)];
      const double vel = a.lActVelocity / velocity_scale_;
      any_error |= a.bError;
      power_on |= a.bControlLoopClosed;
      const bool near_zero = std::isfinite(vel) && std::fabs(vel) <= 0.001;
      stale_job |= a.bHasJob && !a.bNotMoving && near_zero && !a.bError;
      axis_motion_unresolved |= !near_zero || !a.bNotMoving || a.bHasJob ||
        a.bMovingFw || a.bMovingBw;
      out << " | Axis" << axis << ":ready=" << a.bControlLoopClosed
          << ",nm=" << a.bNotMoving << ",job=" << a.bHasJob
          << ",fw=" << a.bMovingFw << ",bw=" << a.bMovingBw
          << ",v=" << vel << ",error=" << a.bError
          << ",code=0x" << std::hex << static_cast<uint32_t>(a.nErrorCode) << std::dec;
      if (a.bControlLoopClosed) {
        FunctionStatus stop{}, move{};
        const bool stop_ok = read_stop_status(axis, stop);
        const bool move_ok = read_move_status(axis, move);
        out << ",stop=" << (stop_ok ? (stop.bBusy ? "BUSY" : (stop.bError ? "ERROR" : "IDLE")) : "UNREADABLE")
            << ",stop_done=" << (stop_ok ? static_cast<int>(stop.bDone) : -1)
            << ",move=" << (move_ok ? (move.bError ? "ERROR" : (move.bBusy ? "BUSY" : "IDLE")) : "UNREADABLE");
        stop_fb_blocked |= !stop_ok || !move_ok || stop.bBusy || stop.bError || move.bError;
      }
    }
    if (ads_recovery_required_) {
      out << " | NEXT: ADS reconnect approval only after verified NC idle and safe power mode.";
    } else if (any_error) {
      out << " | NEXT: Inspect real drive fault and safe standstill; /agv/reset_faults, then approval if requested.";
    } else if (stale_job) {
      out << " | NEXT: Stale NC job suspected. /agv/recover_stale_jobs ONLY after physical stop confirmed; then /agv/reset_faults.";
    } else if (axis_motion_unresolved || stop_fb_blocked) {
      out << " | NEXT: NC/FB state unresolved. Do not infer safety from zero velocity; use independent safety stop and PLC service.";
    } else if (reset_rearm_pending_) {
      out << " | NEXT: Reset pending; inspect /agv_status and approve power restore ONLY if requested.";
    } else if (!motion_ready_) {
      out << " | NEXT: NC idle; inspect power mode, latch and startup logs. Reset may be required.";
    } else {
      out << " | NEXT: Driver READY. New operator command required.";
    }
    out << " | TwinCAT Runtime STOP is NOT a safety stop. No restart requested.";
    response->success = true; // diagnostic was successfully obtained, not motion clearance
    response->message = out.str();
  }

  // /agv/reset_faults is a NON-BLOCKING request.  The service ACK means
  // Reset has been submitted, NOT that the PLC has re-armed the motor.
  // The 20Hz timer will confirm every axis is fault-free and safely stopped.
  void reset_faults_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (ads_recovery_required_) {
      response->success = false;
      response->message = "ADS reconnect recovery locked; use /agv/approve_ads_recovery after PLC idle verification.";
      return;
    }
    if (diagnostic_read_only_) {
      response->success = false;
      response->message = "DIAGNOSTIC READ-ONLY: PLC Power/Reset writes are disabled.";
      return;
    }
    if (mock_mode_) {
      fault_latched_ = false;
      safe_stop_latched_ = false;
      safe_stop_active_ = false;
      motion_ready_ = true;
      reset_rearm_pending_ = false;
      response->success = true;
      response->message = "Mock mode: reset/re-arm complete.";
      return;
    }
    if (!ads_connected_) {
      response->success = false;
      response->message = "ADS disconnected. Reset rejected.";
      return;
    }
    if (stale_recovery_pending_) {
      response->success = false;
      response->message = "Operator stale-job STOP recovery still pending; watch logs.";
      return;
    }
    if (reset_rearm_pending_) {
      response->success = true;
      response->message = "Reset already submitted; awaiting PLC standstill/FB release. Check /agv_status.";
      return;
    }
    std::array<AxisStatus, 3> status{};
    if (!read_axis_status(status)) {
      response->success = false;
      response->message = "No fresh AxisStatus. No Reset sent.";
      return;
    }
    // One-key recovery: when the NC still owns a job, do NOT issue MC_Reset,
    // MC_Stop or change drive power. Remain latched and observe for the job
    // to be released by the PLC / independently protected maintenance action.
    // Once idle is verified, the same request continues into normal re-arm.
    if (!has_any_fault(status)) {
      int blocked_axis = 0;
      for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
        const auto & ax = status[axis_index(axis)];
        if (ax.bHasJob || !ax.bNotMoving || ax.bMovingFw || ax.bMovingBw) {
          blocked_axis = axis;
          break;
        }
      }
      if (blocked_axis != 0) {
        clear_motion_requests();
        motion_ready_ = false;
        safe_stop_latched_ = true;
        safe_stop_active_ = false;
        safe_stop_reason_ = "NC job held Axis" + std::to_string(blocked_axis);
        reset_rearm_pending_ = true;
        reset_waiting_for_nc_release_ = true;
        reset_waiting_nc_started_ = std::chrono::steady_clock::now();
        reset_standstill_verifying_ = false;
        RCLCPP_ERROR(this->get_logger(),
          "[RECOVERY WAIT NC] Axis%d has a retained job or unresolved motion. "
          "Motion locked; no STOP, Power cycle or TwinCAT restart issued.", blocked_axis);
        response->success = true;
        response->message = "Recovery monitoring started: Axis" +
          std::to_string(blocked_axis) +
          " has retained NC job. Clear the mechanical/safety cause and release "
          "the job in PLC; the driver will finish re-arm automatically when "
          "verified idle. No motor power cycling attempted.";
        return;
      }
    }
    clear_motion_requests();
    motion_ready_ = false;
    // Latch BEFORE issuing Reset so neither the normal STOP handler nor
    // the watchdog can transmit StopMove on a faulted axis.
    fault_latched_ = fault_latched_ || has_any_fault(status);
    safe_stop_latched_ = true;
    safe_stop_active_ = false;
    if (safe_stop_reason_.empty()) safe_stop_reason_ = "Reset/re-arm in progress";
    bool ok = true;
    // MC_Stop can keep a fault-free axis locked after Done=1.  Beckhoff
    // documents MC_Reset as a possible lock release.  Do NOT Reset a
    // fault-free axis while HasJob=1, or while any axis is still moving.
    const bool all_still = all_axes_physically_stopped(status);
    for (const int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & st = status[axis_index(axis)];
      const bool real_fault = st.bError;
      bool completed_stop_lock = false;
      if (!real_fault && all_still && !st.bHasJob && st.bControlLoopClosed) {
        FunctionStatus fb{};
        completed_stop_lock = read_stop_status(axis, fb) &&
          fb.bBusy && fb.bDone && !fb.bError;
      }
      if (!real_fault && !completed_stop_lock) continue;
      RCLCPP_WARN(this->get_logger(),
        "[FAULT RESET TX] Axis%d Reset once (reason=%s; no new MoveVelocity)",
        axis, real_fault ? "real axis fault" : "completed MC_Stop lock");
      int32_t result = UNKNOWN_ERR_TcMCFUN;
      const bool sent = plc_fb_reader_.send_reset_once(axis, result);
      fault_reset_attempted_[static_cast<std::size_t>(axis)] = true;
      if (!sent) {
        RCLCPP_ERROR(this->get_logger(),
          "[FAULT RESET] Axis%d PLC method rejected: 0x%X", axis,
          static_cast<unsigned int>(result));
        ok = false;
      }
    }
    if (!ok) {
      response->success = false;
      response->message = "PLC Reset rejected. Motion remains latched. Check drive diagnostics.";
      return;
    }
    reset_rearm_pending_ = true;
    reset_waiting_for_nc_release_ = false;
    reset_power_restore_sent_ = false;
    reset_waiting_for_approval_ = false;
    power_restore_approved_ = false;
    reset_rearm_started_ = std::chrono::steady_clock::now();
    reset_standstill_started_ = {};
    reset_standstill_verifying_ = false;
    response->success = true;
    response->message = "Reset submitted. Verify PLC idle continuously for 3s; automatic READY only if power state already matches. Check /agv_status.";
  }

  // Operator confirmation is separate from clearing a drive alarm.
  // Calling this only authorizes the guarded re-enable attempt in the timer;
  // it does not issue a power or velocity command in this callback.
  void approve_power_restore_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (ads_recovery_required_ || diagnostic_read_only_ || mock_mode_ || !ads_connected_ ||
        !reset_rearm_pending_ || !reset_waiting_for_approval_) {
      response->success = false;
      response->message = "No reset recovery awaiting operator power approval.";
      return;
    }
    std::array<AxisStatus, 3> status{};
    if (!read_axis_status(status) || has_any_fault(status) ||
        !all_axes_physically_stopped(status)) {
      response->success = false;
      response->message = "PLC fault/standstill verification failed; approval rejected.";
      return;
    }
    power_restore_approved_ = true;
    reset_rearm_started_ = std::chrono::steady_clock::now();
    response->success = true;
    response->message = "Power restoration approved; monitoring PLC feedback, no automatic motion.";
  }

  // Runs on the SAME executor thread as motion control. ADS RPCs elsewhere
  // remain synchronous; a PLC-side independent watchdog is mandatory.
  void process_pending_reset(const std::array<AxisStatus, 3> & status)
  {
    if (!reset_rearm_pending_) return;
    if (reset_waiting_for_nc_release_) {
      // This is a passive observation state. A near-zero encoder velocity with
      // HasJob=1 is NOT standstill; never bypass this PLC interlock.
      bool idle = !has_any_fault(status);
      for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
        const auto & ax = status[axis_index(axis)];
        const double vel = std::fabs(ax.lActVelocity / velocity_scale_);
        if (ax.bHasJob || !ax.bNotMoving || ax.bMovingFw || ax.bMovingBw ||
            !std::isfinite(vel) || vel > stop_suppress_velocity_mps_) {
          idle = false;
        }
        if (idle && ax.bControlLoopClosed) {
          FunctionStatus stop{}, move{};
          if (!read_stop_status(axis, stop) || !read_move_status(axis, move) ||
              stop.bError || stop.bBusy || move.bError) idle = false;
        }
      }
      if (!idle) {
        const double wait_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - reset_waiting_nc_started_).count();
        if (wait_s >= kNcReleaseWaitTimeoutSec) {
          reset_waiting_for_nc_release_ = false;
          reset_rearm_pending_ = false;
          RCLCPP_ERROR(this->get_logger(),
            "[RECOVERY BLOCKED] NC job not released in %.0fs. "
            "Motion remains locked. Check PLC/drive and safety stop; no TwinCAT restart.",
            kNcReleaseWaitTimeoutSec);
        }
        return;
      }
      reset_waiting_for_nc_release_ = false;
      reset_rearm_started_ = std::chrono::steady_clock::now();
      reset_standstill_verifying_ = false;
      RCLCPP_INFO(this->get_logger(),
        "[RECOVERY NC CLEARED] All axes idle; verifying before READY.");
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - reset_rearm_started_).count();
    if (elapsed >= reset_recovery_timeout_sec_ &&
        (!reset_waiting_for_approval_ || power_restore_approved_)) {
      reset_standstill_verifying_ = false;
      reset_rearm_pending_ = false;
      RCLCPP_ERROR(this->get_logger(),
        "[CRITICAL] Reset verification timeout (%.1fs). Motion LOCKED. "
        "TwinCAT restart NOT triggered: independent STO/safe-stop feedback unavailable. "
        "Inspect NC jobs/drive state; use supervised recovery.",
        reset_recovery_timeout_sec_);
      return;
    }
    if (has_any_fault(status) || !all_axes_physically_stopped(status)) {
      reset_standstill_verifying_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[FAULT RESET WAIT] axis fault or bNotMoving/velocity mismatch; NO motion");
      return;
    }
    const bool left_on = status[axis_index(left_wheel_axis_)].bControlLoopClosed;
    const bool right_on = status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    const bool rail_on = status[axis_index(rail_axis_)].bControlLoopClosed;
    // Dual mode never forces a rail-vs-wheel enable switch during reset.
    // Restore only the current, internally consistent power configuration.
    const bool wheels_wanted = allow_simultaneous_drive_ ? left_on : wheel_mode_requested();
    const bool rail_wanted = allow_simultaneous_drive_ ? rail_on : rail_mode_requested();
    if ((allow_simultaneous_drive_ && (left_on != right_on || (!left_on && !rail_on))) ||
        left_on != wheels_wanted || right_on != wheels_wanted ||
        rail_on != rail_wanted) {
      // Expected after vendor STOP fallback: the failed axis was PowerDisabled.
      // Only after errors are clear AND every axis is physically stationary do
      // we restore the requested drive group. No MoveVelocity is sent here.
      if (!all_axes_physically_stopped(status)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[FAULT RESET WAIT] Power mismatch after Disable, but standstill not confirmed; NO PowerEnable");
        return;
      }
      // Do not restore PowerEnable in the same operation as MC_Reset.
      // The operator must explicitly approve via /agv/approve_power_restore.
      // An alarm clear does not authorize automatic servo torque restoration.
      // Explicit operator authorization is required whenever an axis is OFF.
      reset_standstill_verifying_ = false;
      if (!power_restore_approved_) {
        if (!reset_waiting_for_approval_) {
          reset_waiting_for_approval_ = true;
          RCLCPP_WARN(this->get_logger(),
            "[RESET WAIT OPERATOR] Alarm clear, axis stationary, Power mismatch. "
            "Verify machine/zone, then call /agv/approve_power_restore.");
        }
        return;
      }
      if (!reset_power_restore_sent_) {
        bool ok = true;
        if (!allow_simultaneous_drive_) {
          if (wheels_wanted) {
            if (rail_on) ok = power_disable_axis(rail_axis_) && ok;
            if (!left_on) ok = power_enable_axis(left_wheel_axis_) && ok;
            if (!right_on) ok = power_enable_axis(right_wheel_axis_) && ok;
          } else if (rail_wanted) {
            if (left_on) ok = power_disable_axis(left_wheel_axis_) && ok;
            if (right_on) ok = power_disable_axis(right_wheel_axis_) && ok;
            if (!rail_on) ok = power_enable_axis(rail_axis_) && ok;
          }
        } else {
          // Dual mode: restore only the axes that were already logically requested.
          if (wheels_wanted && !left_on) ok = power_enable_axis(left_wheel_axis_) && ok;
          if (wheels_wanted && !right_on) ok = power_enable_axis(right_wheel_axis_) && ok;
          if (rail_wanted && !rail_on) ok = power_enable_axis(rail_axis_) && ok;
        }
        reset_power_restore_sent_ = true;
        axis_snapshot_valid_ = false;
        if (!ok) {
          RCLCPP_ERROR(this->get_logger(),
            "[FAULT RESET] Power restore command failed; motion stays locked.");
        } else {
          RCLCPP_WARN(this->get_logger(),
            "[FAULT RESET] Alarm clear + standstill confirmed; restoring required PowerEnable state.");
        }
      }
      return; // next timer snapshot must confirm Ready feedback before re-arm
    }
    if (!plc_idle_for_existing_power(status) || !all_previous_stops_settled()) {
      reset_standstill_verifying_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[FAULT RESET WAIT] PLC Stop/Move FB not quiescent; NO motion");
      return;
    }
    // Require continuous clean snapshots for >=3 s. No timeout-based
    // assumption of standstill; any failed check restarts the window.
    const auto verified_now = std::chrono::steady_clock::now();
    if (!reset_standstill_verifying_) {
      reset_standstill_started_ = verified_now;
      reset_standstill_verifying_ = true;
      RCLCPP_DEBUG(this->get_logger(),
        "[RESET VERIFY] PLC fault-free, standstill and FB idle. Verifying %.1fs continuously.",
        reset_standstill_verify_sec_);
      return;
    }
    const double stable_sec = std::chrono::duration<double>(
      verified_now - reset_standstill_started_).count();
    if (stable_sec < reset_standstill_verify_sec_) return;
    reset_standstill_verifying_ = false;
    reset_rearm_pending_ = false;
    reset_power_restore_sent_ = false;
    reset_waiting_for_approval_ = false;
    power_restore_approved_ = false;
    fallback_disable_attempted_.fill(false);
    fault_disable_attempted_.fill(false);
    power_transition_stage_ = PowerTransitionStage::NONE;
    power_stage_command_sent_ = false;
    fault_latched_ = false;
    fault_stop_active_ = false;
    safe_stop_latched_ = false;
    safe_stop_active_ = false;
    safe_stop_reason_.clear();
    fault_reset_attempted_.fill(false);
    stop_attempt_initialized_.fill(false);
    last_stop_tx_initialized_.fill(false);
    stop_idle_confirm_initialized_.fill(false);
    move_attempt_initialized_.fill(false);
    last_accepted_move_initialized_.fill(false);
    move_feedback_stable_initialized_.fill(false);
    wheel_motion_armed_ = false;
    rail_motion_armed_ = false;
    wheel_axes_enabled_ = left_on && right_on;
    rail_axis_enabled_ = rail_on;
    motion_ready_ = true;
    last_cmd_vel_time_ = this->now();
    last_rail_cmd_time_ = this->now();
    last_cmd_vel_steady_ = std::chrono::steady_clock::now();
    last_rail_cmd_steady_ = last_cmd_vel_steady_;
    RCLCPP_INFO(this->get_logger(),
      "[FAULT RESET COMPLETE] %.1fs continuous PLC idle confirmed. "
      "Fresh velocity input required before motion.", reset_standstill_verify_sec_);
  }

  // ============================================================
  // Beckhoff environmental sensor
  // ============================================================
  // Environment polling runs on its OWN ADS port and thread. Never use the
  // motor-control ADS connection from this thread (vendor CTcMC is not thread-safe).
  // Only successful ADS reads may be published; no default-zero samples.
  void start_environment_sensor_polling()
  {
    if (!enable_environment_sensor_ || mock_mode_ || environment_worker_.joinable()) return;
    environment_stop_.store(false);
    environment_worker_ = std::thread([this]() { environment_poll_worker(); });
    RCLCPP_INFO(this->get_logger(),
      "[ENV] Independent ADS polling started: Sensor_Modbus.SensorInfo (1 Hz)");
  }

  void stop_environment_polling()
  {
    environment_stop_.store(true);
    if (environment_worker_.joinable()) environment_worker_.join();
  }

  void invalidate_environment_sample()
  {
    std::lock_guard<std::mutex> lock(environment_mutex_);
    environment_sample_valid_ = false;
  }

  void environment_poll_worker()
  {
    const AmsNetId remote_net_id REMOTE_NET_ID;
    AmsAddr addr{};
    addr.netId = remote_net_id;
    addr.port = AMSPORT_R0_PLC_TC3;
    long port = 0;
    uint32_t handle = 0;
    int error_streak = 0;

    const auto close_port = [&]() {
      if (port > 0) {
        if (handle != 0) {
          (void)AdsSyncWriteReqEx(port, &addr, ADSIGRP_SYM_RELEASEHND,
            0, sizeof(handle), &handle);
        }
        AdsPortCloseEx(port);
      }
      handle = 0;
      port = 0;
    };

    while (!environment_stop_.load()) {
      const auto started = std::chrono::steady_clock::now();
      if (port <= 0) {
        port = AdsPortOpenEx();
        if (port > 0 && AdsSyncSetTimeoutEx(port, 500) != 0) close_port();
      }
      if (port > 0 && handle == 0) {
        constexpr char symbol[] = "Sensor_Modbus.SensorInfo";
        uint32_t received = 0;
        const long rc = AdsSyncReadWriteReqEx2(
          port, &addr, ADSIGRP_SYM_HNDBYNAME, 0,
          sizeof(handle), &handle, sizeof(symbol),
          const_cast<char *>(symbol), &received);
        if (rc != 0 || received != sizeof(handle) || handle == 0) {
          handle = 0;
          if (++error_streak == 1 || error_streak % 10 == 0) {
            RCLCPP_WARN(this->get_logger(),
              "[ENV] ADS symbol handle acquisition failed (rc=%ld, streak=%d)",
              rc, error_streak);
          }
          close_port();
        }
      }
      if (port > 0 && handle != 0) {
        SensorInfo sample{};
        uint32_t received = 0;
        const long rc = AdsSyncReadReqEx2(
          port, &addr, ADSIGRP_SYM_VALBYHND, handle,
          sizeof(sample), &sample, &received);
        const float solar = static_cast<float>(sample.nSolar_Radiation);
        const float temp = static_cast<float>(sample.rTemperature);
        const float humidity = static_cast<float>(sample.rHumidity);
        const float co2 = static_cast<float>(sample.nCO2_PPM2);
        if (rc == 0 && received == sizeof(sample) &&
            std::isfinite(solar) && std::isfinite(temp) &&
            std::isfinite(humidity) && std::isfinite(co2)) {
          {
            std::lock_guard<std::mutex> lock(environment_mutex_);
            environment_sensor_data_ = sample;
            environment_sample_time_ = std::chrono::steady_clock::now();
            environment_sample_valid_ = true;
            ++environment_sample_sequence_;
          }
          if (error_streak > 0) {
            RCLCPP_INFO(this->get_logger(), "[ENV] ADS sensor polling recovered");
          }
          error_streak = 0;
        } else {
          invalidate_environment_sample();
          if (++error_streak == 1 || error_streak % 10 == 0) {
            RCLCPP_WARN(this->get_logger(),
              "[ENV] ADS sensor read failed (rc=%ld bytes=%u expected=%zu streak=%d)",
              rc, received, sizeof(sample), error_streak);
          }
          close_port();  // TwinCAT RUN restart may invalidate symbol handles.
        }
      }
      // Interruptible 1 Hz cadence; stop within ~100 ms plus ADS timeout.
      while (!environment_stop_.load() &&
             std::chrono::steady_clock::now() - started < 1s) {
        std::this_thread::sleep_for(100ms);
      }
    }
    close_port();
  }

  void publish_environment_sensor_if_due(const rclcpp::Time & now)
  {
    if (!enable_environment_sensor_ || mock_mode_ || !ads_connected_) return;
    if ((now - last_environment_publish_time_).seconds() < environment_publish_period_sec_) return;
    last_environment_publish_time_ = now;

    SensorInfo sample{};
    uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(environment_mutex_);
      if (!environment_sample_valid_ ||
          std::chrono::steady_clock::now() - environment_sample_time_ > 3s ||
          environment_sample_sequence_ == environment_published_sequence_) {
        return;
      }
      sample = environment_sensor_data_;
      sequence = environment_sample_sequence_;
      environment_published_sequence_ = sequence;
    }
    const float solar = static_cast<float>(sample.nSolar_Radiation);
    const float temperature = static_cast<float>(sample.rTemperature);
    const float humidity = static_cast<float>(sample.rHumidity);
    const float co2 = static_cast<float>(sample.nCO2_PPM2);

    std_msgs::msg::Float32 solar_msg;
    solar_msg.data = solar;
    environment_solar_pub_->publish(solar_msg);
    std_msgs::msg::Float32 temperature_msg;
    temperature_msg.data = temperature;
    environment_temperature_pub_->publish(temperature_msg);
    std_msgs::msg::Float32 humidity_msg;
    humidity_msg.data = humidity;
    environment_humidity_pub_->publish(humidity_msg);
    std_msgs::msg::Float32 co2_msg;
    co2_msg.data = co2;
    environment_co2_pub_->publish(co2_msg);
    std_msgs::msg::Float32MultiArray data_msg;
    data_msg.data = {solar, temperature, humidity, co2};
    environment_data_pub_->publish(data_msg);

    if (log_environment_sensor_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "[ENV] seq=%llu solar=%.1f W/m2 temp=%.2f C humidity=%.2f%% CO2=%.0f ppm",
        static_cast<unsigned long long>(sequence), static_cast<double>(solar),
        static_cast<double>(temperature), static_cast<double>(humidity),
        static_cast<double>(co2));
    }
  }

  // ADS reconnect safety: only recover transport. No motor control writes.
  // IMPORTANT: Vendor Disconnect() must preserve its internal method-map and
  // release notifications BEFORE destroying the underlying AdsDevice.
  void mark_ads_connection_lost(const char * reason)
  {
    if (ads_recovery_required_) return;
    ads_recovery_required_ = true;
    ads_reconnect_transport_needed_ = true;
    motion_ready_ = false;
    safe_stop_latched_ = true;
    safe_stop_active_ = false;  // STOP must not be sent over broken ADS.
    safe_stop_reason_ = "ADS link lost / reconnect requires operator approval";
    clear_motion_requests();
    wheel_motion_armed_ = false;
    rail_motion_armed_ = false;
    reset_rearm_pending_ = false;
    reset_waiting_for_nc_release_ = false;
    reset_standstill_verifying_ = false;
    stale_recovery_pending_ = false;
    power_transition_stage_ = PowerTransitionStage::NONE;
    axis_snapshot_valid_ = false;
    invalidate_environment_sample();
    ads_failure_count_ = 0;
    ads_fb_failure_count_ = 0;
    RCLCPP_ERROR(this->get_logger(),
      "[ADS LINK LOST] %s. All motion disabled in ROS. "
      "Hardware/PLC watchdog is required. Reconnect will NEVER auto-enable motors.", reason);
  }

  void reconnect_ads_transport_if_due()
  {
    if (!ads_auto_reconnect_ || !ads_reconnect_transport_needed_ || shutting_down_) return;
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_ads_reconnect_tp_).count()
        < ads_reconnect_interval_sec_) return;
    last_ads_reconnect_tp_ = now;
    RCLCPP_WARN(this->get_logger(),
      "[ADS RECONNECT] Closing stale ADS handles and opening new transport. "
      "All PLC motion/power commands remain blocked.");

    // Close the read-only FB RPC port, vendor notification(s), then vendor
    // command route. No ROS velocity, STOP, Reset or Power command is sent.
    plc_fb_reader_.close();
    if (ads_connected_) {
      Disconnect();
      ads_connected_ = false;
    }
    axis_snapshot_valid_ = false;
    invalidate_environment_sample();

    int32_t result = UNKNOWN_ERR_TcMCFUN;
    try { result = Connect(); }
    catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "[ADS RECONNECT] Vendor Connect exception: %s", e.what());
    }
    catch (...) { RCLCPP_ERROR(this->get_logger(), "[ADS RECONNECT] Unknown Connect exception"); }
    if (result != NO_ERR_TcMCFUN) {
      // A partial vendor Connect may leave its local port allocated.
      Disconnect();
      RCLCPP_WARN(this->get_logger(),
        "[ADS RECONNECT] Vendor Connect failed (0x%X); will retry later.",
        static_cast<unsigned>(result));
      return;
    }
    ads_connected_ = true;
    const AmsNetId plc_net_id REMOTE_NET_ID;
    if (!plc_fb_reader_.open(plc_net_id)) {
      RCLCPP_ERROR(this->get_logger(),
        "[ADS RECONNECT] PLC FB status handles unavailable; retrying later.");
      Disconnect();
      ads_connected_ = false;
      return;
    }
    std::array<AxisStatus, 3> state{};
    axis_snapshot_valid_ = false;
    if (!read_axis_status(state)) {
      RCLCPP_ERROR(this->get_logger(),
        "[ADS RECONNECT] Axis status unavailable; retrying later.");
      plc_fb_reader_.close();
      Disconnect();
      ads_connected_ = false;
      return;
    }
    ads_reconnect_transport_needed_ = false;
    ads_failure_count_ = 0;
    ads_fb_failure_count_ = 0;
    // Do not call configure_power_for_current_state_blocking(): it would
    // PowerEnable automatically. Do not clear safe_stop_latched_.
    start_environment_sensor_polling();
    publish_axis_diagnostics(state);
    RCLCPP_WARN(this->get_logger(),
      "[ADS RECONNECTED] Fresh ADS transport/PLC symbols available. "
      "Motion LOCKED. Inspect /agv/axis_diagnostics; then call "
      "/agv/approve_ads_recovery only after physical safety verification.");
  }

  void approve_ads_recovery_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (mock_mode_ || diagnostic_read_only_ || !ads_recovery_required_ ||
        ads_reconnect_transport_needed_ || !ads_connected_ ||
        reset_rearm_pending_ || stale_recovery_pending_ || fault_latched_) {
      response->success = false;
      response->message = "ADS recovery approval denied: transport/fault/recovery not ready.";
      return;
    }
    std::array<AxisStatus, 3> status{};
    axis_snapshot_valid_ = false;
    if (!read_axis_status(status) || has_any_fault(status)) {
      response->success = false;
      response->message = "ADS approval denied: PLC axes not readable or faulted.";
      return;
    }
    // Stronger than actual velocity alone: ALL axes must have no retained jobs,
    // no direction bit, and genuine PLC bNotMoving. No Power state transition.
    for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto & a = status[axis_index(axis)];
      if (!a.bNotMoving || a.bHasJob || a.bMovingFw || a.bMovingBw ||
          !std::isfinite(a.lActVelocity) ||
          std::fabs(a.lActVelocity / velocity_scale_) > stop_suppress_velocity_mps_) {
        response->success = false;
        response->message = "ADS approval denied: PLC job/direction/standstill unresolved.";
        return;
      }
    }
    const bool left_on = status[axis_index(left_wheel_axis_)].bControlLoopClosed;
    const bool right_on = status[axis_index(right_wheel_axis_)].bControlLoopClosed;
    const bool rail_on = status[axis_index(rail_axis_)].bControlLoopClosed;
    if (left_on != right_on ||
        (!allow_simultaneous_drive_ &&
         (left_on != wheel_mode_requested() || rail_on != rail_mode_requested())) ||
        (allow_simultaneous_drive_ && !left_on && !rail_on) ||
        !plc_idle_for_existing_power(status)) {
      response->success = false;
      response->message = "ADS approval denied: PLC Power mode or Stop/Move FB mismatch. "
        "No automatic Power changes; consult PLC operator.";
      return;
    }
    // Clear every stale per-command gate. Fresh teleop messages are required.
    clear_motion_requests();
    stop_attempt_initialized_.fill(false);
    last_stop_tx_initialized_.fill(false);
    stop_idle_confirm_initialized_.fill(false);
    last_accepted_move_initialized_.fill(false);
    move_feedback_stable_initialized_.fill(false);
    move_attempt_initialized_.fill(false);
    wheel_motion_armed_ = false;
    rail_motion_armed_ = false;
    wheel_axes_enabled_ = left_on && right_on;
    rail_axis_enabled_ = rail_on;
    last_cmd_vel_steady_ = std::chrono::steady_clock::now();
    last_rail_cmd_steady_ = last_cmd_vel_steady_;
    ads_recovery_required_ = false;
    safe_stop_latched_ = false;
    safe_stop_active_ = false;
    safe_stop_reason_.clear();
    motion_ready_ = true;
    response->success = true;
    response->message = "ADS recovery approved: verified PLC idle, NO Power/Move sent; "
      "new commands now permitted. Keep operator at E-stop.";
    RCLCPP_WARN(this->get_logger(), "[ADS RECOVERY APPROVED] No Power/Move command sent.");
  }

  // ============================================================
  // Main timer
  // ============================================================
  void timer_callback()
  {
    const auto now = this->now();
    if (!mock_mode_ && ads_reconnect_transport_needed_) {
      reconnect_ads_transport_if_due();
      if ((status_counter_++ % 20) == 0) publish_status();
      return; // No motor/PLC method calls during transport recovery.
    }
    if (diagnostic_read_only_ && !mock_mode_) {
      // Read-only monitoring is deliberately slower than the motion timer.
      // No PLC power/move/stop/reset methods are sent by this path.
      const auto steady_now = std::chrono::steady_clock::now();
      if (!last_diagnostic_poll_initialized_ ||
          std::chrono::duration<double>(steady_now - last_diagnostic_poll_tp_).count()
            >= diagnostic_ads_poll_period_sec_) {
        last_diagnostic_poll_initialized_ = true;
        last_diagnostic_poll_tp_ = steady_now;
        std::array<AxisStatus, 3> snapshot{};
        if (ads_connected_ && read_axis_status(snapshot)) {
          ads_failure_count_ = 0;
          publish_axis_diagnostics(snapshot);
          log_axis_monitor(snapshot);
        } else if (ads_connected_ && ++ads_failure_count_ >= ads_disconnect_failure_count_) {
          mark_ads_connection_lost("consecutive diagnostic AxisStatus read failures");
        }
      }
      publish_environment_sensor_if_due(now);
      if ((status_counter_++ % 20) == 0) publish_status();
      return;
    }

    if (mock_mode_) {
      publish_mock_feedback();
    } else if (ads_connected_) {
      std::array<AxisStatus, 3> status{};
      if (read_axis_status(status)) {
        // Also probe PLC FB RPC handle: stale symbols can survive a RUN restart
        // even when the vendor axis-status handle still appears readable.
        bool fb_ok = true;
        const auto chk_now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(chk_now - last_ads_health_tp_).count() >= 1.0) {
          last_ads_health_tp_ = chk_now;
          FunctionStatus probe{};
          fb_ok = plc_fb_reader_.read_stop(left_wheel_axis_, probe);
        }
        if (!fb_ok) {
          // Missing PLC FB status is NOT permission to move or use an old handle.
          motion_ready_ = false;
          safe_stop_latched_ = true;
          safe_stop_active_ = false;
          safe_stop_reason_ = "PLC FB handle unreadable; motion locked";
          clear_motion_requests();
          if (++ads_fb_failure_count_ >= ads_disconnect_failure_count_)
            mark_ads_connection_lost("PLC FB status handle failed after ADS reconnect/restart");
        } else {
          ads_fb_failure_count_ = 0;
        }
        ads_failure_count_ = 0;
        if (ads_reconnect_transport_needed_) {
          if ((status_counter_++ % 20) == 0) publish_status();
          return;
        }
        publish_axis_diagnostics(status);
        process_axis_status(status);
        if (stale_recovery_pending_) process_stale_recovery(status);
        if (reset_rearm_pending_) process_pending_reset(status);
        if (safe_stop_latched_) {
          process_safe_stop(status);
        }
        if (motion_ready_ && !fault_latched_ && !safe_stop_latched_) {
          process_power_transition(status);
        }
      } else {
        // First missing ADS sample blocks motion immediately; waiting for
        // multiple errors is ONLY for deciding when to recreate the transport.
        motion_ready_ = false;
        safe_stop_latched_ = true;
        safe_stop_active_ = false;
        safe_stop_reason_ = "ADS feedback missing; motion locked";
        clear_motion_requests();
        wheel_motion_armed_ = false;
        rail_motion_armed_ = false;
        if (++ads_failure_count_ >= ads_disconnect_failure_count_) {
          mark_ads_connection_lost("consecutive ADS AxisStatus read failures");
        }
      }
    }

    double target_linear = requested_linear_x_mps_;
    double target_angular = requested_angular_z_rps_;
    double target_rail = requested_rail_speed_mps_;

    // Command timeout is a NORMAL-OPERATION watchdog. Fault recovery is
    // Reset-only and must not dispatch StopMove through the timeout path.
    // A command-loss watchdog must use monotonic time, not ROS /clock or wall time.
    // It still requires a responsive executor; PLC-side protection is mandatory.
    const auto watchdog_now = std::chrono::steady_clock::now();
    const bool wheel_timeout =
      !fault_latched_ &&
      std::chrono::duration<double>(watchdog_now - last_cmd_vel_steady_).count()
        > cmd_vel_timeout_sec_;
    const bool rail_timeout =
      !fault_latched_ &&
      std::chrono::duration<double>(watchdog_now - last_rail_cmd_steady_).count()
        > rail_cmd_timeout_sec_;

    if (wheel_timeout) {
      target_linear = 0.0;
      target_angular = 0.0;
      requested_linear_x_mps_ = 0.0;
      requested_angular_z_rps_ = 0.0;

      left_pending_target_mps_ = 0.0;
      right_pending_target_mps_ = 0.0;
      left_waiting_for_stop_ = false;
      right_waiting_for_stop_ = false;

      if (!wheel_watchdog_triggered_) {
        wheel_watchdog_triggered_ = true;
        RCLCPP_DEBUG(
          this->get_logger(),
          "[WATCHDOG] /cmd_vel timeout %.1f sec -> WHEEL STOP",
          cmd_vel_timeout_sec_);
        // Communication watchdog is a recoverable stop.  Do not latch: Nav2 /
        // twist_mux can legitimately have a short command gap.  A fresh
        // /cmd_vel command may resume motion after the axes stop.
      }
    } else {
      wheel_watchdog_triggered_ = false;
    }

    if (rail_timeout) {
      target_rail = 0.0;
      requested_rail_speed_mps_ = 0.0;

      rail_pending_target_mps_ = 0.0;
      rail_waiting_for_stop_ = false;
      rail_stop_hold_active_ = false;

      if (!rail_watchdog_triggered_) {
        rail_watchdog_triggered_ = true;
        RCLCPP_DEBUG(
          this->get_logger(),
          "[WATCHDOG] /rail/cmd_speed timeout %.1f sec -> RAIL STOP",
          rail_cmd_timeout_sec_);
        // Recoverable rail watchdog stop; do not latch SAFE STOP.  A fresh
        // rail command may resume after the axis is stopped.
      }
    } else {
      rail_watchdog_triggered_ = false;
    }

    const bool power_transition_active =
      power_transition_stage_ != PowerTransitionStage::NONE;

    if (mock_mode_) {
      if (power_transition_active) {
        apply_mock_commands(0.0, 0.0, 0.0);
      } else if (allow_simultaneous_drive_) {
        apply_mock_commands(wheel_axes_enabled_ ? target_linear : 0.0,
          wheel_axes_enabled_ ? target_angular : 0.0,
          rail_axis_enabled_ ? target_rail : 0.0);
      } else if (rail_mode_requested()) {
        apply_mock_commands(0.0, 0.0, target_rail);
      } else {
        apply_mock_commands(target_linear, target_angular, 0.0);
      }
    } else if (ads_connected_ && motion_ready_ && !fault_latched_ && !safe_stop_latched_) {
      if (!power_transition_active &&
          (allow_simultaneous_drive_ || wheel_mode_requested()) && wheel_axes_enabled_) {
        apply_wheel_command(target_linear, target_angular);
      } else {
        applied_left_mps_ = 0.0;
        applied_right_mps_ = 0.0;
      }

      if (!power_transition_active &&
          (allow_simultaneous_drive_ || rail_mode_requested()) && rail_axis_enabled_) {
        const double permitted_rail = (allow_simultaneous_drive_ && wheel_axes_enabled_) ?
          std::clamp(target_rail, -dual_rail_max_mps_, dual_rail_max_mps_) : target_rail;
        apply_rail_command(permitted_rail);
      } else {
        applied_rail_mps_ = 0.0;
      }
    } else {
      applied_left_mps_ = 0.0;
      applied_right_mps_ = 0.0;
      applied_rail_mps_ = 0.0;
    }

    // Environment data is supplied asynchronously by the vendor ADS
    // notification.  This call does not perform a synchronous ADS read.
    publish_environment_sensor_if_due(now);

    if ((status_counter_++ % 20) == 0) {
      publish_status();
    }
  }

  // ============================================================
  // Axis feedback
  // ============================================================
  bool read_axis_status(std::array<AxisStatus, 3> & status)
  {
    if (!ads_connected_) return false;
    const auto now = std::chrono::steady_clock::now();
    // Prevent repeated synchronous ADS round trips inside one 20Hz cycle.
    // Unlike the vendor Notification callback, this cache is only accessed
    // from the ROS executor thread, avoiding unsynchronized memcpy reads.
    if (axis_snapshot_valid_ &&
        std::chrono::duration<double, std::milli>(
          now - axis_snapshot_time_).count() < 35.0) {
      status = axis_snapshot_;
      return true;
    }
    std::array<AxisStatus, 3> snapshot{};
    const auto ads_read_started = std::chrono::steady_clock::now();
    const int32_t rc = ReadAxisStatus(snapshot.data(), 3);
    const double ads_read_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - ads_read_started).count();
    if (ads_read_ms > ads_slow_read_warn_ms_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[ADS SLOW READ] ReadAxisStatus took %.1fms (warning > %.1fms). "
        "Synchronous ADS blocks the executor; diagnostic mode reduces polling "
        "but cannot enforce an ADS request timeout.",
        ads_read_ms, ads_slow_read_warn_ms_);
    }
    if (rc != NO_ERR_TcMCFUN) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[ADS READ] ReadAxisStatus failed: 0x%X. No motion authorization.",
        static_cast<unsigned int>(rc));
      axis_snapshot_valid_ = false;
      return false;
    }
    for (const auto & s : snapshot) {
      if (!std::isfinite(s.lActVelocity) || !std::isfinite(s.lActPosition)) {
        axis_snapshot_valid_ = false;
        return false;
      }
    }
    axis_snapshot_ = snapshot;
    axis_snapshot_time_ = std::chrono::steady_clock::now();
    axis_snapshot_valid_ = true;
    status = snapshot;
    return true;
  }

  bool has_any_fault(const std::array<AxisStatus, 3> & status) const
  {
    return
      status[axis_index(left_wheel_axis_)].bError ||
      status[axis_index(right_wheel_axis_)].bError ||
      status[axis_index(rail_axis_)].bError;
  }

  void log_axis_monitor(const std::array<AxisStatus, 3> & status)
  {
    if (!log_axis_monitor_) {
      return;
    }

    const auto now = this->now();
    if ((now - last_axis_monitor_time_).seconds() < axis_monitor_period_sec_) {
      return;
    }
    last_axis_monitor_time_ = now;

    const auto & left =
      status[axis_index(left_wheel_axis_)];
    const auto & right =
      status[axis_index(right_wheel_axis_)];
    const auto & rail =
      status[axis_index(rail_axis_)];

    const double left_actual_mps =
      (left.lActVelocity / velocity_scale_) * left_wheel_sign_;
    const double right_actual_mps =
      (right.lActVelocity / velocity_scale_) * right_wheel_sign_;
    const double rail_actual_mps =
      (rail.lActVelocity / velocity_scale_) * rail_sign_;

    const double left_cmd_mps =
      requested_linear_x_mps_ -
      requested_angular_z_rps_ * wheel_separation_m_ / 2.0;
    const double right_cmd_mps =
      requested_linear_x_mps_ +
      requested_angular_z_rps_ * wheel_separation_m_ / 2.0;

    RCLCPP_INFO(
      this->get_logger(),
      "[AXIS MONITOR] "
      "CMD L=%+.3f R=%+.3f m/s | "
      "ACT L=%+.3f R=%+.3f m/s | "
      "READY L=%d R=%d | NM L=%d R=%d | JOB L=%d R=%d | "
      "ERR L=%d(0x%X) R=%d(0x%X) | "
      "RAIL cmd=%+.3f act=%+.3f ready=%d nm=%d err=%d(0x%X)",
      left_cmd_mps,
      right_cmd_mps,
      left_actual_mps,
      right_actual_mps,
      left.bControlLoopClosed ? 1 : 0,
      right.bControlLoopClosed ? 1 : 0,
      left.bNotMoving ? 1 : 0,
      right.bNotMoving ? 1 : 0,
      left.bHasJob ? 1 : 0,
      right.bHasJob ? 1 : 0,
      left.bError ? 1 : 0,
      static_cast<unsigned int>(left.nErrorCode),
      right.bError ? 1 : 0,
      static_cast<unsigned int>(right.nErrorCode),
      requested_rail_speed_mps_,
      rail_actual_mps,
      rail.bControlLoopClosed ? 1 : 0,
      rail.bNotMoving ? 1 : 0,
      rail.bError ? 1 : 0,
      static_cast<unsigned int>(rail.nErrorCode));

    // Extra warning when either drive wheel is losing speed badly
    // while a non-zero command is still being requested.
    constexpr double kCommandCheckMinMps = 0.02;
    constexpr double kBadTrackingRatio = 0.50;

    if (motion_ready_ && wheel_axes_enabled_ &&
        !safe_stop_latched_ && !fault_latched_ &&
        last_accepted_move_initialized_[static_cast<std::size_t>(left_wheel_axis_)] &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
          last_accepted_move_tp_[static_cast<std::size_t>(left_wheel_axis_)]).count() >
          (wheel_ramp_ms_ / 1000.0 + 0.25) &&
        std::fabs(applied_left_mps_) >= kCommandCheckMinMps &&
        std::fabs(left_cmd_mps) >= kCommandCheckMinMps) {
      const double ratio =
        std::fabs(left_actual_mps) / std::fabs(left_cmd_mps);
      if (ratio < kBadTrackingRatio) {
        RCLCPP_WARN(
          this->get_logger(),
          "[AXIS TRACKING] LEFT actual speed is only %.0f%% of command "
          "(cmd=%+.3f, act=%+.3f m/s)",
          ratio * 100.0,
          left_cmd_mps,
          left_actual_mps);
      }
    }

    if (motion_ready_ && wheel_axes_enabled_ &&
        !safe_stop_latched_ && !fault_latched_ &&
        last_accepted_move_initialized_[static_cast<std::size_t>(right_wheel_axis_)] &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
          last_accepted_move_tp_[static_cast<std::size_t>(right_wheel_axis_)]).count() >
          (wheel_ramp_ms_ / 1000.0 + 0.25) &&
        std::fabs(applied_right_mps_) >= kCommandCheckMinMps &&
        std::fabs(right_cmd_mps) >= kCommandCheckMinMps) {
      const double ratio =
        std::fabs(right_actual_mps) / std::fabs(right_cmd_mps);
      if (ratio < kBadTrackingRatio) {
        RCLCPP_WARN(
          this->get_logger(),
          "[AXIS TRACKING] RIGHT actual speed is only %.0f%% of command "
          "(cmd=%+.3f, act=%+.3f m/s)",
          ratio * 100.0,
          right_cmd_mps,
          right_actual_mps);
      }
    }
  }

  // Read-only axis diagnostics for the external monitoring GUI.
  // Values come directly from PLC AxisStatus. Actual velocity is mm/s -> m/s only.
  void publish_axis_diagnostics(const std::array<AxisStatus, 3> & axes)
  {
    if (!axis_diagnostics_pub_) return;
    const auto now = std::chrono::steady_clock::now();
    if (axis_diag_published_ &&
        std::chrono::duration<double>(now - axis_diag_last_tp_).count() < 0.2) return;
    axis_diag_published_ = true;
    axis_diag_last_tp_ = now;
    std::ostringstream out;
    out << "{\"axes\":[";
    for (int axis = 1; axis <= 3; ++axis) {
      if (axis > 1) out << ',';
      const auto & a = axes[axis_index(axis)];
      const double sign = axis == left_wheel_axis_ ? left_wheel_sign_ :
        (axis == right_wheel_axis_ ? right_wheel_sign_ : rail_sign_);
      out << "{\"axis\":" << axis
          << ",\"velocity_mps\":" << (a.lActVelocity / velocity_scale_ * sign)
          << ",\"position_mm\":" << a.lActPosition
          << ",\"ready\":" << (a.bControlLoopClosed ? "true" : "false")
          << ",\"not_moving\":" << (a.bNotMoving ? "true" : "false")
          << ",\"has_job\":" << (a.bHasJob ? "true" : "false")
          << ",\"moving_fw\":" << (a.bMovingFw ? "true" : "false")
          << ",\"moving_bw\":" << (a.bMovingBw ? "true" : "false")
          << ",\"error\":" << (a.bError ? "true" : "false")
          << ",\"error_code\":" << static_cast<uint32_t>(a.nErrorCode)
          << '}';
    }
    out << "]}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    axis_diagnostics_pub_->publish(msg);
  }

  // Fault response: inhibit new motion immediately, then remove drive enable
  // exactly once for each axis that is ALREADY verified stationary and job-free.
  // A PowerDisable while moving can remove active braking torque; PLC STO / SS1
  // must handle a moving or indeterminate axis independently of ROS/ADS.
  void power_disable_stopped_axes_on_fault(const std::array<AxisStatus, 3> & axes)
  {
    if (diagnostic_read_only_ || !ads_connected_ || shutting_down_) return;
    for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto idx = static_cast<std::size_t>(axis);
      const auto & a = axes[idx - 1];
      if (!a.bControlLoopClosed || fault_disable_attempted_[idx]) continue;
      const double speed_mps = std::fabs(a.lActVelocity / velocity_scale_);
      const bool stationary = a.bNotMoving && !a.bHasJob &&
        !a.bMovingFw && !a.bMovingBw && std::isfinite(speed_mps) &&
        speed_mps <= stop_suppress_velocity_mps_;
      if (!stationary) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[FAULT DISABLE DEFERRED] Axis%d not verified stationary: NM=%d Job=%d "
          "FW=%d BW=%d speed=%.5f. Independent PLC safe stop/STO required.",
          axis, a.bNotMoving ? 1 : 0, a.bHasJob ? 1 : 0,
          a.bMovingFw ? 1 : 0, a.bMovingBw ? 1 : 0, speed_mps);
        continue;
      }
      fault_disable_attempted_[idx] = true; // one shot, before RPC
      RCLCPP_ERROR(this->get_logger(),
        "[FAULT DISABLE TX] Axis%d verified stationary/job-free; requesting PowerDisable.", axis);
      const int32_t result = PowerDisable(static_cast<uint32_t>(axis));
      axis_snapshot_valid_ = false;
      if (result != NO_ERR_TcMCFUN) {
        RCLCPP_ERROR(this->get_logger(),
          "[FAULT DISABLE FAILED] Axis%d code=0x%X; maintain fault latch and inspect PLC.",
          axis, static_cast<unsigned int>(result));
      }
    }
  }

  void process_axis_status(const std::array<AxisStatus, 3> & status)
  {
    log_axis_monitor(status);

    if (has_any_fault(status)) {
      const auto & left =
        status[axis_index(left_wheel_axis_)];
      const auto & right =
        status[axis_index(right_wheel_axis_)];
      const auto & rail =
        status[axis_index(rail_axis_)];

      if (!fault_latched_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[AXIS FAULT] L=%d(0x%X) R=%d(0x%X) Rail=%d(0x%X). "
          "Inhibiting ALL new motion. StopMove is PROHIBITED during fault. "
          "Reset errors first, then verify standstill before re-arm.",
          left.bError ? 1 : 0, static_cast<unsigned int>(left.nErrorCode),
          right.bError ? 1 : 0, static_cast<unsigned int>(right.nErrorCode),
          rail.bError ? 1 : 0, static_cast<unsigned int>(rail.nErrorCode));
        enter_safe_stop("axis fault: Reset-only; no StopMove");
      }

      fault_latched_ = true;
      // Fault on any axis immediately inhibits all ROS motion. Attempt power
      // removal of axes whose motion/NC state is independently verified idle.
      power_disable_stopped_axes_on_fault(status);
      fault_stop_active_ = false;
      safe_stop_active_ = false;  // Generic SAFE STOP sends StopMove: disable it.
      // Even on drive fault, preserve the measured motion in ROS odometry.
      // This path only runs after a successful finite ReadAxisStatus().
      const auto & lf = status[axis_index(left_wheel_axis_)];
      const auto & rf = status[axis_index(right_wheel_axis_)];
      const auto & ra = status[axis_index(rail_axis_)];
      actual_left_mps_ = lf.lActVelocity / velocity_scale_ * left_wheel_sign_;
      actual_right_mps_ = rf.lActVelocity / velocity_scale_ * right_wheel_sign_;
      actual_rail_mps_ = ra.lActVelocity / velocity_scale_ * rail_sign_;
      publish_wheel_feedback(actual_left_mps_, actual_right_mps_);
      publish_rail_feedback(actual_rail_mps_);
      integrate_and_publish_wheel_odom(actual_left_mps_, actual_right_mps_);
      publish_joint_states(actual_left_mps_, actual_right_mps_);
      motion_ready_ = false;
      wheel_stop_pending_ = false;
      clear_motion_requests();

      // Optional and strictly one-shot. Disabled by default: a real drive
      // fault may require operator intervention before Reset is permissible.
      // Even if Reset succeeds, SAFE STOP remains latched until the user
      // calls /agv/reset_faults and standstill/power state are verified.
      if (auto_reset_axis_fault_) {
        for (const int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
          const std::size_t idx = static_cast<std::size_t>(axis);
          if (!status[idx - 1].bError || fault_reset_attempted_[idx]) {
            continue;
          }
          fault_reset_attempted_[idx] = true;
          RCLCPP_WARN(this->get_logger(),
            "[AUTO FAULT RESET TX] Axis%d Reset() once; no StopMove", axis);
          if (diagnostic_read_only_) return;
          const int32_t result = Reset(static_cast<uint32_t>(axis));
          if (result != NO_ERR_TcMCFUN) {
            RCLCPP_ERROR(this->get_logger(),
              "[AUTO FAULT RESET] Axis%d failed: 0x%X (no retry)", axis,
              static_cast<unsigned int>(result));
          }
        }
      }
      return;
    }

    const auto & left_status =
      status[axis_index(left_wheel_axis_)];
    const auto & right_status =
      status[axis_index(right_wheel_axis_)];
    const auto & rail_status =
      status[axis_index(rail_axis_)];

    const bool left_ready_now = left_status.bControlLoopClosed;
    const bool right_ready_now = right_status.bControlLoopClosed;
    const bool rail_ready_now = rail_status.bControlLoopClosed;

    if (!axis_state_initialized_) {
      prev_left_ready_ = left_ready_now;
      prev_right_ready_ = right_ready_now;
      prev_rail_ready_ = rail_ready_now;
      prev_left_error_ = left_status.bError;
      prev_right_error_ = right_status.bError;
      prev_rail_error_ = rail_status.bError;
      prev_left_error_code_ = static_cast<uint32_t>(left_status.nErrorCode);
      prev_right_error_code_ = static_cast<uint32_t>(right_status.nErrorCode);
      prev_rail_error_code_ = static_cast<uint32_t>(rail_status.nErrorCode);
      axis_state_initialized_ = true;
    } else {
      if (left_ready_now != prev_left_ready_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[AXIS STATE] LEFT Ready %d -> %d",
          prev_left_ready_ ? 1 : 0,
          left_ready_now ? 1 : 0);
        prev_left_ready_ = left_ready_now;
      }

      if (right_ready_now != prev_right_ready_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[AXIS STATE] RIGHT Ready %d -> %d",
          prev_right_ready_ ? 1 : 0,
          right_ready_now ? 1 : 0);
        prev_right_ready_ = right_ready_now;
      }

      if (rail_ready_now != prev_rail_ready_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[AXIS STATE] RAIL Ready %d -> %d",
          prev_rail_ready_ ? 1 : 0,
          rail_ready_now ? 1 : 0);
        prev_rail_ready_ = rail_ready_now;
      }

      const uint32_t left_code =
        static_cast<uint32_t>(left_status.nErrorCode);
      const uint32_t right_code =
        static_cast<uint32_t>(right_status.nErrorCode);
      const uint32_t rail_code =
        static_cast<uint32_t>(rail_status.nErrorCode);

      if (left_status.bError != prev_left_error_ ||
          left_code != prev_left_error_code_)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "[AXIS STATE] LEFT error=%d code=0x%X",
          left_status.bError ? 1 : 0,
          static_cast<unsigned int>(left_code));
        prev_left_error_ = left_status.bError;
        prev_left_error_code_ = left_code;
      }

      if (right_status.bError != prev_right_error_ ||
          right_code != prev_right_error_code_)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "[AXIS STATE] RIGHT error=%d code=0x%X",
          right_status.bError ? 1 : 0,
          static_cast<unsigned int>(right_code));
        prev_right_error_ = right_status.bError;
        prev_right_error_code_ = right_code;
      }

      if (rail_status.bError != prev_rail_error_ ||
          rail_code != prev_rail_error_code_)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "[AXIS STATE] RAIL error=%d code=0x%X",
          rail_status.bError ? 1 : 0,
          static_cast<unsigned int>(rail_code));
        prev_rail_error_ = rail_status.bError;
        prev_rail_error_code_ = rail_code;
      }
    }

    wheel_axes_enabled_ =
      left_status.bControlLoopClosed && right_status.bControlLoopClosed;
    rail_axis_enabled_ = rail_status.bControlLoopClosed;

    for (int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const auto idx = static_cast<std::size_t>(axis);
      if (fallback_disable_attempted_[idx] && status[idx - 1].bControlLoopClosed) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[STOP FALLBACK UNCONFIRMED] Axis%d STILL ENABLED after PowerDisable; "
          "hardware E-stop / PLC operator intervention required", axis);
      }
    }

    const double left_mps =
      (left_status.lActVelocity / velocity_scale_) * left_wheel_sign_;
    const double right_mps =
      (right_status.lActVelocity / velocity_scale_) * right_wheel_sign_;
    const double rail_mps =
      (rail_status.lActVelocity / velocity_scale_) * rail_sign_;

    actual_left_mps_ = left_mps;
    actual_right_mps_ = right_mps;
    actual_rail_mps_ = rail_mps;

    publish_wheel_feedback(left_mps, right_mps);
    publish_rail_feedback(rail_mps);
    integrate_and_publish_wheel_odom(left_mps, right_mps);
    publish_joint_states(left_mps, right_mps);
  }

  // Negative/positive direction from signed PLC encoder feedback and last
  // accepted MoveVelocity. A stall may have velocity=0 while HasJob=1.
  // A request in the opposite direction must NEVER skip STOP because
  // software applied_mps was cleared by an earlier zero-speed command.
  bool axis_reverse_requires_stop(
    int axis, const AxisStatus & fb, double target_mps, double sign) const
  {
    constexpr double kDirEpsilon = 0.001;
    if (std::fabs(target_mps) <= kDirEpsilon) return false;
    const double actual = (fb.lActVelocity / velocity_scale_) * sign;
    const bool nc_idle = fb.bNotMoving && !fb.bHasJob &&
      !fb.bMovingFw && !fb.bMovingBw &&
      std::fabs(actual) <= stopped_velocity_threshold_mps_;
    if (nc_idle) return false;
    if (std::fabs(actual) > kDirEpsilon && target_mps * actual < 0.0) return true;
    // Only use directional PLC flags if feedback sign is well defined.
    const bool moving_positive = sign > 0 ? fb.bMovingFw : fb.bMovingBw;
    const bool moving_negative = sign > 0 ? fb.bMovingBw : fb.bMovingFw;
    if (target_mps < -kDirEpsilon && moving_positive) return true;
    if (target_mps > kDirEpsilon && moving_negative) return true;
    // The PLC may briefly show no job while NC still reports moving.
    // Compare against last accepted velocity until a genuine idle state.
    if (axis >= 1 && axis <= 3) {
      const auto idx = static_cast<std::size_t>(axis);
      if (last_accepted_move_initialized_[idx] &&
          std::fabs(last_accepted_target_mps_[idx]) > kDirEpsilon &&
          target_mps * last_accepted_target_mps_[idx] < 0.0) return true;
    }
    return false;
  }

  static double wrap_yaw(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!imu_correction_enable_) return;
    const auto & q = msg->orientation;
    const double n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    const bool orientation_known = msg->orientation_covariance[0] >= 0.0;
    const bool gyro_known = msg->angular_velocity_covariance[0] >= 0.0;
    if (!orientation_known || !gyro_known || !std::isfinite(n2) || n2 < 0.25 || n2 > 2.25 ||
        !std::isfinite(msg->angular_velocity.z) || !std::isfinite(q.x) ||
        !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
      imu_valid_ = false;
      return;
    }
    const double n = std::sqrt(n2);
    const double x=q.x/n, y=q.y/n, z=q.z/n, w=q.w/n;
    imu_yaw_rad_ = wrap_yaw(imu_yaw_sign_ *
      std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0*(y*y + z*z)));
    imu_yaw_rate_rps_ = imu_yaw_sign_ * msg->angular_velocity.z;
    imu_last_rx_tp_ = std::chrono::steady_clock::now();
    imu_valid_ = true;
  }

  void reset_imu_correction()
  {
    imu_straight_active_ = false;
    imu_angular_correction_rps_ = 0.0;
    imu_eval_initialized_ = false;
  }

  // Only update the PLC steering offset after BOTH previous commands have had
  // time to settle. IMU callback does not transmit any ADS commands.
  double imu_steering_correction(double linear_mps, double angular_rps)
  {
    const auto now = std::chrono::steady_clock::now();
    const bool fresh = imu_valid_ &&
      std::chrono::duration<double>(now - imu_last_rx_tp_).count() <= imu_timeout_sec_;
    const bool stopped = std::fabs(linear_mps) < 1.0e-6 &&
      std::fabs(angular_rps) < 1.0e-6;
    const bool near_straight = std::fabs(angular_rps) <= imu_straight_angular_threshold_rps_;
    const bool straight = near_straight &&
      std::fabs(linear_mps) >= imu_min_wheel_speed_mps_;
    if (!imu_correction_enable_ || !fresh || stopped ||
        wheel_direction_change_active_ || wheel_stop_pending_ ||
        left_waiting_for_stop_ || right_waiting_for_stop_) {
      if (imu_angular_correction_rps_ != 0.0 && !fresh) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[IMU] stale/invalid data; steering correction disabled");
      }
      reset_imu_correction();
      return 0.0;
    }
    if (straight && !imu_straight_active_) {
      // Every fresh straight segment gets a new hold-heading reference.
      imu_target_yaw_rad_ = imu_yaw_rad_;
      imu_straight_active_ = true;
      imu_angular_correction_rps_ = 0.0;
      imu_eval_initialized_ = true;
      imu_last_eval_tp_ = now;
      RCLCPP_INFO(this->get_logger(), "[IMU HEADING] reference yaw=%+.3f rad", imu_target_yaw_rad_);
      return 0.0;
    }
    if (!straight && imu_straight_active_) {
      imu_straight_active_ = false;
      imu_angular_correction_rps_ = 0.0;
      imu_eval_initialized_ = false;
    }

    const auto li = static_cast<std::size_t>(left_wheel_axis_);
    const auto ri = static_cast<std::size_t>(right_wheel_axis_);
    const double settle_s = std::max(wheel_balance_settle_sec_,
      (wheel_ramp_ms_ + ads_command_settle_extra_ms_) / 1000.0 + 0.25);
    const bool pair_settled = last_accepted_move_initialized_[li] &&
      last_accepted_move_initialized_[ri] &&
      std::chrono::duration<double>(now-last_accepted_move_tp_[li]).count() >= settle_s &&
      std::chrono::duration<double>(now-last_accepted_move_tp_[ri]).count() >= settle_s;
    if (pair_settled && (!imu_eval_initialized_ ||
        std::chrono::duration<double>(now-imu_last_eval_tp_).count() >=
          std::max(settle_s, imu_correction_interval_sec_))) {
      const double yaw_error = straight ? wrap_yaw(imu_target_yaw_rad_ - imu_yaw_rad_) : 0.0;
      const double raw = straight ?
        imu_heading_kp_*yaw_error - imu_heading_kd_*imu_yaw_rate_rps_ :
        imu_turn_kp_*(angular_rps - imu_yaw_rate_rps_);
      const double original_left = linear_mps - angular_rps*wheel_separation_m_/2.0;
      const double original_right = linear_mps + angular_rps*wheel_separation_m_/2.0;
      // Never let a heading correction reverse a wheel or introduce a
      // nonzero command on a deliberately stopped inside wheel.
      const double axis_margin = 0.35 * std::min(std::fabs(original_left), std::fabs(original_right));
      const double max_corr = std::min(imu_max_angular_correction_rps_,
        2.0*axis_margin/wheel_separation_m_);
      // Keep the ultimate steering authority, but prevent an abrupt jump
      // such as -0.05 -> +0.05 rad/s on a single PLC update.
      // This is applied only when the prior Axis1/2 commands are settled.
      const double desired_corr = std::clamp(raw, -max_corr, max_corr);
      const double previous_corr = imu_angular_correction_rps_;
      const double step = std::min(imu_max_correction_step_rps_, max_corr);
      imu_angular_correction_rps_ = std::clamp(
        desired_corr, previous_corr - step, previous_corr + step);
      // Preserve the current wheel-direction guard even if operating limits
      // shrink between updates (e.g. a change from forward to curved motion).
      imu_angular_correction_rps_ = std::clamp(
        imu_angular_correction_rps_, -max_corr, max_corr);
      imu_last_eval_tp_ = now;
      imu_eval_initialized_ = true;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
        "[IMU CONTROL] mode=%s yaw=%+.3f ref=%+.3f rate=%+.3f target_rate=%+.3f raw=%+.4f desired=%+.4f prev=%+.4f corr=%+.4f step_max=%.4f rad/s",
        straight ? "HEADING" : "RATE", imu_yaw_rad_, imu_target_yaw_rad_,
        imu_yaw_rate_rps_, angular_rps, raw, desired_corr, previous_corr,
        imu_angular_correction_rps_, step);
    }
    return imu_angular_correction_rps_;
  }

  // ============================================================
  // Wheel control
  // ============================================================
  void reset_wheel_balance()
  {
    wheel_balance_error_filtered_mps_ = 0.0;
    wheel_balance_initialized_ = false;
    wheel_balance_correction_mps_ = 0.0;
    wheel_balance_reference_initialized_ = false;
    wheel_balance_eval_initialized_ = false;
    wheel_balance_stable_tracking_ = false;
    wheel_balance_reverse_tracking_ = false;
    wheel_balance_release_active_ = false;
    wheel_balance_tick_initialized_ = false;
    wheel_balance_reference_left_mps_ = 0.0;
    wheel_balance_reference_right_mps_ = 0.0;
  }

  void apply_wheel_balance(
    double & left_target_mps,
    double & right_target_mps)
  {
    // Balance the measured PLC wheel velocities, including reverse and pivot turns.
    // The incoming signed targets are the UNCORRECTED differential-drive setpoints.
    // For curves, compare measured/commanded speed ratios rather than forcing
    // unequal target wheel speeds to become equal.
    const double base_left = left_target_mps;
    const double base_right = right_target_mps;
    const double left_nominal = std::fabs(base_left);
    const double right_nominal = std::fabs(base_right);
    if (!wheel_balance_enable_ ||
        left_nominal < wheel_balance_min_speed_mps_ ||
        right_nominal < wheel_balance_min_speed_mps_ ||
        wheel_direction_change_active_ || wheel_stop_pending_ ||
        left_waiting_for_stop_ || right_waiting_for_stop_) {
      reset_wheel_balance();
      return;
    }

    // Reset the feedback filter and previous correction on a change in either
    // intended wheel target (including a direction change / pivot turn).
    if (!wheel_balance_reference_initialized_ ||
        std::fabs(base_left - wheel_balance_reference_left_mps_) >
          wheel_command_epsilon_mps_ ||
        std::fabs(base_right - wheel_balance_reference_right_mps_) >
          wheel_command_epsilon_mps_) {
      reset_wheel_balance();
      wheel_balance_reference_initialized_ = true;
      wheel_balance_reference_left_mps_ = base_left;
      wheel_balance_reference_right_mps_ = base_right;
      return;
    }

    const auto li = static_cast<std::size_t>(left_wheel_axis_);
    const auto ri = static_cast<std::size_t>(right_wheel_axis_);
    const auto now = std::chrono::steady_clock::now();
    const double settle_s = std::max(wheel_balance_settle_sec_,
      (wheel_ramp_ms_ + ads_command_settle_extra_ms_) / 1000.0 + 0.25);

    // A command ACK only confirms method acceptance, not that the wheel
    // reached the new setpoint. Require BOTH wheels' latest accepted commands
    // to age past the ramp/settling window before computing a new correction.
    const bool pair_has_moves =
      last_accepted_move_initialized_[li] && last_accepted_move_initialized_[ri];
    const bool pair_settled = pair_has_moves &&
      std::chrono::duration<double>(now - last_accepted_move_tp_[li]).count() >= settle_s &&
      std::chrono::duration<double>(now - last_accepted_move_tp_[ri]).count() >= settle_s;

    const double left_actual = actual_left_mps_;
    const double right_actual = actual_right_mps_;
    const double left_sign = base_left >= 0.0 ? 1.0 : -1.0;
    const double right_sign = base_right >= 0.0 ? 1.0 : -1.0;
    const double min_feedback = std::max(0.010, wheel_balance_min_speed_mps_ * 0.40);
    const bool feedback_ok = std::isfinite(left_actual) &&
      std::isfinite(right_actual) &&
      left_actual * left_sign >= min_feedback &&
      right_actual * right_sign >= min_feedback;

    if (!feedback_ok) {
      // Do not boost a blocked/stalled wheel or use opposite-direction feedback.
      reset_wheel_balance();
      wheel_balance_reference_initialized_ = true;
      wheel_balance_reference_left_mps_ = base_left;
      wheel_balance_reference_right_mps_ = base_right;
      return;
    }

    // Equal-magnitude commands (straight / reverse / pivot): this is simply
    // abs(left_actual) - abs(right_actual). For unequal curve commands,
    // normalize each measured magnitude by its own intended magnitude.
    const double nominal_mean = 0.5 * (left_nominal + right_nominal);
    const double left_tracking = std::fabs(left_actual) / left_nominal;
    const double right_tracking = std::fabs(right_actual) / right_nominal;
    const double raw_error = (left_tracking - right_tracking) * nominal_mean;
    const double dt = wheel_balance_tick_initialized_ ?
      std::clamp(std::chrono::duration<double>(now - wheel_balance_last_tick_tp_).count(),
                 0.0, 0.10) : 0.0;
    wheel_balance_last_tick_tp_ = now;
    wheel_balance_tick_initialized_ = true;

    // Stable band: start a timer only after continuously staying in deadband.
    // A single close-to-zero sample must NOT cancel a useful correction.
    if (std::fabs(raw_error) <= wheel_balance_deadband_mps_) {
      if (!wheel_balance_stable_tracking_) {
        wheel_balance_stable_tracking_ = true;
        wheel_balance_stable_since_ = now;
      }
    } else {
      wheel_balance_stable_tracking_ = false;
      wheel_balance_release_active_ = false;
    }

    const bool stable_confirmed = wheel_balance_stable_tracking_ &&
      std::chrono::duration<double>(now - wheel_balance_stable_since_).count() >=
        wheel_balance_stable_hold_sec_;
    if (stable_confirmed && !wheel_balance_release_active_) {
      wheel_balance_release_active_ = true;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[WHEEL BALANCE HOLD] error within %.3f m/s for %.2fs; gradually releasing",
        wheel_balance_deadband_mps_, wheel_balance_stable_hold_sec_);
    }

    // Opposite SIGN (with real magnitude) must persist briefly before
    // dropping a stale correction, otherwise noisy zero crossings chatter.
    const bool clear_reverse =
      std::fabs(raw_error) > wheel_balance_deadband_mps_ &&
      wheel_balance_correction_mps_ * raw_error < 0.0;
    if (clear_reverse) {
      if (!wheel_balance_reverse_tracking_) {
        wheel_balance_reverse_tracking_ = true;
        wheel_balance_reverse_since_ = now;
      }
    } else {
      wheel_balance_reverse_tracking_ = false;
    }
    const bool reverse_confirmed = wheel_balance_reverse_tracking_ &&
      std::chrono::duration<double>(now - wheel_balance_reverse_since_).count() >=
        wheel_balance_reverse_hold_sec_;
    if (reverse_confirmed) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[WHEEL BALANCE REVERSE] current_error=%+.3f prev_correction=%+.3f; "
        "dropping stale bias after %.2fs sustained reversal",
        raw_error, wheel_balance_correction_mps_, wheel_balance_reverse_hold_sec_);
      wheel_balance_correction_mps_ = 0.0;
      wheel_balance_initialized_ = false;
      wheel_balance_error_filtered_mps_ = 0.0;
      wheel_balance_reverse_tracking_ = false;
      wheel_balance_eval_initialized_ = true;
      wheel_balance_last_eval_tp_ = now;
      // Keep the base targets. No opposite-sign correction until the PLC
      // confirms both wheel commands have settled again.
      return;
    }

    if (wheel_balance_release_active_) {
      const double decrement = wheel_balance_release_rate_mps2_ * dt;
      if (wheel_balance_correction_mps_ > 0.0) {
        wheel_balance_correction_mps_ =
          std::max(0.0, wheel_balance_correction_mps_ - decrement);
      } else {
        wheel_balance_correction_mps_ =
          std::min(0.0, wheel_balance_correction_mps_ + decrement);
      }
      if (wheel_balance_correction_mps_ == 0.0) {
        wheel_balance_initialized_ = false;
        wheel_balance_error_filtered_mps_ = 0.0;
      }
    } else {
      const bool evaluation_due = !wheel_balance_eval_initialized_ ||
        std::chrono::duration<double>(now - wheel_balance_last_eval_tp_).count() >= settle_s;
      if (pair_settled && evaluation_due && !wheel_balance_stable_tracking_) {
        wheel_balance_last_eval_tp_ = now;
        wheel_balance_eval_initialized_ = true;
        // Avoid using the old filter sign when the raw mismatch has flipped.
        if (!wheel_balance_initialized_ ||
            raw_error * wheel_balance_error_filtered_mps_ <= 0.0) {
          wheel_balance_error_filtered_mps_ = raw_error;
          wheel_balance_initialized_ = true;
        } else {
          wheel_balance_error_filtered_mps_ =
            wheel_balance_filter_alpha_ * raw_error +
            (1.0 - wheel_balance_filter_alpha_) * wheel_balance_error_filtered_mps_;
        }
        const double max_correction = std::min(
          wheel_balance_max_correction_mps_, std::min(left_nominal, right_nominal) * 0.40);
        wheel_balance_correction_mps_ = std::clamp(
          wheel_balance_kp_ * wheel_balance_error_filtered_mps_,
          -max_correction, max_correction);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[WHEEL BALANCE] target L=%+.3f R=%+.3f actual L=%+.3f R=%+.3f "
          "tracking L=%.3f R=%.3f err=%+.3f corr=%+.3f settle=%.2fs",
          base_left, base_right, left_actual, right_actual,
          left_tracking, right_tracking, raw_error,
          wheel_balance_correction_mps_, settle_s);
      }
    }

    // Hold a bounded correction during PLC ramp/settle; keep original wheel
    // directions and proportional target ratio on curved paths. Never cross 0.
    const double relative_correction = wheel_balance_correction_mps_ / nominal_mean;
    left_target_mps = left_sign * left_nominal * (1.0 - relative_correction);
    right_target_mps = right_sign * right_nominal * (1.0 + relative_correction);
  }

  void apply_wheel_command(double linear_mps, double angular_rps)
  {
    // Final safety barrier: no MoveVelocity path is allowed while a real axis
    // fault / SAFE STOP latch is active. StopMove is handled separately.
    if (fault_latched_ || safe_stop_latched_ || !motion_ready_) {
      return;
    }

    // STOP_PENDING: if either wheel StopMove returned BUSY, the PLC may still
    // be executing the previous velocity. Do not send any new MoveVelocity.
    // Keep stopping both wheels and resume only after both measured speeds are
    // near zero. The newest /cmd_vel remains stored for the next cycle.
    if (wheel_stop_pending_) {
      // A near-zero encoder speed is NOT sufficient: NC can still retain a
      // MoveVelocity job which resumes when an obstruction is removed.
      std::array<AxisStatus, 3> pending_axes{};
      if (!read_axis_status(pending_axes) || has_any_fault(pending_axes)) {
        enter_safe_stop("STOP_PENDING: unreadable/faulted PLC axis feedback");
        safe_stop_active_ = false; // No blind STOP on unknown feedback.
        return;
      }
      const auto & ls = pending_axes[axis_index(left_wheel_axis_)];
      const auto & rs = pending_axes[axis_index(right_wheel_axis_)];
      const bool left_stopped =
        axis_is_physically_stopped(ls, left_wheel_sign_) &&
        !ls.bHasJob && !ls.bMovingFw && !ls.bMovingBw;
      const bool right_stopped =
        axis_is_physically_stopped(rs, right_wheel_sign_) &&
        !rs.bHasJob && !rs.bMovingFw && !rs.bMovingBw;

      if (!left_stopped) {
        (void)request_axis_stop(
          left_wheel_axis_, wheel_ramp_ms_, "STOP_PENDING LEFT");
      }
      if (!right_stopped) {
        (void)request_axis_stop(
          right_wheel_axis_, wheel_ramp_ms_, "STOP_PENDING RIGHT");
      }

      if (left_stopped && right_stopped) {
        wheel_stop_pending_ = false;
        applied_left_mps_ = 0.0;
        applied_right_mps_ = 0.0;
        left_waiting_for_stop_ = false;
        right_waiting_for_stop_ = false;
        left_pending_target_mps_ = 0.0;
        right_pending_target_mps_ = 0.0;
        RCLCPP_INFO(
          this->get_logger(),
          "[STOP_PENDING] Both wheels stopped. New wheel commands are allowed.");
        return;
      }

      const double pending_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wheel_stop_pending_since_).count();

      if (pending_sec >= wheel_stop_pending_timeout_sec_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[STOP_PENDING] Timeout %.2f sec. L=%.3f m/s R=%.3f m/s. "
          "Escalating to SAFE STOP LATCH.",
          pending_sec, actual_left_mps_, actual_right_mps_);
        enter_safe_stop("wheel STOP_PENDING timeout");
      }
      return;
    }

    // IMU adds a bounded steering delta. It never writes to PLC directly.
    // Balance uses the IMU-adjusted wheel targets as its ratio references,
    // so it cannot cancel an intentionally requested turn.
    const double imu_delta = imu_steering_correction(linear_mps, angular_rps);
    const double controlled_angular = angular_rps + imu_delta;
    double left_target_mps =
      linear_mps - controlled_angular * wheel_separation_m_ / 2.0;
    double right_target_mps =
      linear_mps + controlled_angular * wheel_separation_m_ / 2.0;

    // PLC feedback-based balance for straight, reverse, pivot, and curved motion.
    apply_wheel_balance(left_target_mps, right_target_mps);

    // Clamp ACTUAL wheel axis setpoints (not only Twist.linear), since a
    // steering command can otherwise exceed the dual-drive speed cap.
    if (allow_simultaneous_drive_ && rail_axis_enabled_) {
      left_target_mps = std::clamp(left_target_mps,
        -dual_wheel_axis_max_mps_, dual_wheel_axis_max_mps_);
      right_target_mps = std::clamp(right_target_mps,
        -dual_wheel_axis_max_mps_, dual_wheel_axis_max_mps_);
    }

    // Read the same PLC snapshot for both wheels; do not infer direction from
    // applied_* variables, which are cleared immediately after STOP submission.
    std::array<AxisStatus, 3> reverse_status{};
    if (!read_axis_status(reverse_status) || has_any_fault(reverse_status)) return;
    const auto & rev_left = reverse_status[axis_index(left_wheel_axis_)];
    const auto & rev_right = reverse_status[axis_index(right_wheel_axis_)];
    const bool left_reverse = axis_reverse_requires_stop(
      left_wheel_axis_, rev_left, left_target_mps, left_wheel_sign_);
    const bool right_reverse = axis_reverse_requires_stop(
      right_wheel_axis_, rev_right, right_target_mps, right_wheel_sign_);

    if (wheel_direction_change_active_) {
      wheel_direction_pending_left_mps_ = left_target_mps;
      wheel_direction_pending_right_mps_ = right_target_mps;

      std::array<AxisStatus, 3> dir_status{};
      if (!read_axis_status(dir_status) || has_any_fault(dir_status)) return;
      const auto & ls = dir_status[axis_index(left_wheel_axis_)];
      const auto & rs = dir_status[axis_index(right_wheel_axis_)];
      const bool left_stopped = axis_is_physically_stopped(ls, left_wheel_sign_) &&
        !ls.bHasJob && !ls.bMovingFw && !ls.bMovingBw;
      const bool right_stopped = axis_is_physically_stopped(rs, right_wheel_sign_) &&
        !rs.bHasJob && !rs.bMovingFw && !rs.bMovingBw;

      if (!left_stopped) (void)request_axis_stop(left_wheel_axis_, wheel_ramp_ms_, "wheel pair direction change");
      if (safe_stop_latched_ || fault_latched_) return;
      if (!right_stopped) (void)request_axis_stop(right_wheel_axis_, wheel_ramp_ms_, "wheel pair direction change");
      if (safe_stop_latched_ || fault_latched_) return;
      if (!left_stopped || !right_stopped) {
        wheel_pair_stable_initialized_ = false;
        return;
      }
      if (!previous_stop_window_elapsed(left_wheel_axis_, wheel_ramp_ms_) ||
          !previous_stop_window_elapsed(right_wheel_axis_, wheel_ramp_ms_)) {
        wheel_pair_stable_initialized_ = false;
        return;
      }
      const auto stable_now = std::chrono::steady_clock::now();
      if (!wheel_pair_stable_initialized_) {
        wheel_pair_stable_initialized_ = true;
        wheel_pair_stable_since_ = stable_now;
        return;
      }
      if (std::chrono::duration<double, std::milli>(
          stable_now - wheel_pair_stable_since_).count() < 200.0) return;
      wheel_pair_stable_initialized_ = false;
      // A STOP command received while decelerating cancels the queued reverse.
      if (std::fabs(left_target_mps) < 1.0e-6 &&
          std::fabs(right_target_mps) < 1.0e-6) {
        wheel_direction_change_active_ = false;
        wheel_direction_pending_left_mps_ = 0.0;
        wheel_direction_pending_right_mps_ = 0.0;
        applied_left_mps_ = 0.0;
        applied_right_mps_ = 0.0;
        return;
      }

      applied_left_mps_ = 0.0;
      applied_right_mps_ = 0.0;
      wheel_direction_change_active_ = false;
      left_target_mps = wheel_direction_pending_left_mps_;
      right_target_mps = wheel_direction_pending_right_mps_;
      wheel_direction_pending_left_mps_ = 0.0;
      wheel_direction_pending_right_mps_ = 0.0;
      RCLCPP_INFO(this->get_logger(),
        "[WHEEL DIRECTION] Both wheels stopped; opposite-direction command may now start.");
    } else if (left_reverse || right_reverse) {
      wheel_direction_change_active_ = true;
      wheel_pair_stable_initialized_ = false;
      wheel_direction_pending_left_mps_ = left_target_mps;
      wheel_direction_pending_right_mps_ = right_target_mps;
      RCLCPP_WARN(this->get_logger(),
        "[WHEEL DIRECTION] Sign reversal requested. STOPPING BOTH WHEELS before direction change.");
      (void)request_axis_stop(left_wheel_axis_, wheel_ramp_ms_, "wheel pair direction change");
      if (safe_stop_latched_ || fault_latched_) return;
      (void)request_axis_stop(right_wheel_axis_, wheel_ramp_ms_, "wheel pair direction change");
      return;
    }

    apply_single_axis_velocity(
      left_wheel_axis_, left_target_mps, left_wheel_sign_,
      wheel_ramp_ms_, wheel_command_epsilon_mps_,
      applied_left_mps_, actual_left_mps_,
      left_waiting_for_stop_, left_pending_target_mps_, "LEFT");

    apply_single_axis_velocity(
      right_wheel_axis_, right_target_mps, right_wheel_sign_,
      wheel_ramp_ms_, wheel_command_epsilon_mps_,
      applied_right_mps_, actual_right_mps_,
      right_waiting_for_stop_, right_pending_target_mps_, "RIGHT");
  }

  // ============================================================
  // Rail control
  // ============================================================
  void apply_rail_command(double rail_target_mps)
  {
    // This routine contains a direct StopMove(Axis3) path, so it must also
    // obey the fault interlock independently of request_axis_stop().
    if (fault_latched_ || safe_stop_latched_ || !motion_ready_) {
      return;
    }
    constexpr double kZeroMps = 1.0e-6;
    constexpr double kStoppedThresholdMps = 0.01;

    // ---------------------------------------------------------------------
    // Axis3 rail STOP policy (vendor callback/notification style)
    //
    // 1) AxisStatus is read ONLY from the ADS Notification cache.
    // 2) Do not overlap StopMove with the MoveVelocity ramp.
    // 3) Send StopMove(Axis3) once after the ramp window is clear.
    // 4) After StopMove is accepted, do NOT retransmit it every control cycle.
    //    Wait for Notification bNotMoving + near-zero actual velocity.
    // 5) Explicit STOP (g / 0.0 m/s) affects Axis3 only.
    // 6) Direction reversal uses the same Axis3-only STOP sequence, then
    //    waits rail_direction_stop_hold_ms before the opposite MoveVelocity.
    // ---------------------------------------------------------------------

    std::array<AxisStatus, 3> status{};
    bool rail_b_not_moving = false;
    if (read_axis_status(status)) {
      const auto & rail_status =
        status[axis_index(rail_axis_)];
      rail_b_not_moving = rail_status.bNotMoving;
    }

    const bool rail_has_job = read_axis_status(status) &&
      status[axis_index(rail_axis_)].bHasJob;
    const bool rail_physically_stopped =
      rail_b_not_moving && !rail_has_job &&
      std::fabs(actual_rail_mps_) <= kStoppedThresholdMps;
    const bool rail_velocity_is_zero =
      std::fabs(actual_rail_mps_) <= stop_suppress_velocity_mps_;

    const auto now_tp = std::chrono::steady_clock::now();

    // If a rail STOP sequence is active, keep the newest desired target.
    if (rail_stop_sequence_active_) {
      rail_pending_target_mps_ = rail_target_mps;

      // Explicit STOP must cancel a pending reverse/restart.
      if (std::fabs(rail_target_mps) <= kZeroMps) {
        rail_pending_target_mps_ = 0.0;
      }

      // ------------------------------------------------------------
      // A) StopMove has not been accepted yet.
      //    Respect the MoveVelocity ramp timing before issuing StopMove.
      // ------------------------------------------------------------
      if (!rail_stop_tx_sent_) {
        bool move_ramp_complete = true;

        if (rail_last_move_tx_initialized_) {
          const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
              now_tp - rail_last_move_tx_tp_).count();

          move_ramp_complete = elapsed_ms >= rail_ramp_ms_;
        }

        // No retry loop after BUSY; never StopMove a stationary rail.
        if (rail_physically_stopped) {
          rail_stop_tx_sent_ = true;  // STOP already achieved, wait for state B.
        } else if (rail_velocity_is_zero && !rail_has_job) {
          return;  // No outstanding job: avoid blind STOP.
        } else {
          // STOP is a cancellation command, not a subsequent velocity ramp.
          // Never wait on MoveVelocity Busy/Done to accept a necessary STOP.
          (void)move_ramp_complete; // elapsed ramp is diagnostic only.
          const auto stop_result = request_axis_stop(
            rail_axis_, rail_ramp_ms_, "RAIL STOP sequence");
          if (stop_result == StopRequestResult::ACCEPTED ||
              stop_result == StopRequestResult::ALREADY_REQUESTED) {
            rail_stop_tx_sent_ = true;
            rail_stop_tx_tp_ = now_tp;
          } else if (stop_result == StopRequestResult::BUSY ||
                     stop_result == StopRequestResult::FAILED) {
            // A failed STOP has already caused a no-retry motion latch.
            return;
          } else {
            return;
          }
        }
        // Fall through to B only when the PLC has confirmed standstill;
        // otherwise the next timer cycle observes the STOP status.
        if (!rail_physically_stopped) {return;}
      }

      // ------------------------------------------------------------
      // B) StopMove was accepted.
      //    From here, use Notification only.  Do not send StopMove again.
      // ------------------------------------------------------------
      if (!rail_physically_stopped) {
        return;
      }

      applied_rail_mps_ = 0.0;
      rail_stop_tx_sent_ = false;
      rail_stop_retry_wait_active_ = false;

      // Explicit g/STOP: remain stopped.  No wheel StopMove is issued.
      if (std::fabs(rail_pending_target_mps_) <= kZeroMps) {
        rail_stop_sequence_active_ = false;
        rail_waiting_for_stop_ = false;
        rail_stop_hold_active_ = false;

        RCLCPP_INFO(
          this->get_logger(),
          "[RAIL STOP] Axis%d stopped confirmed by Notification.",
          rail_axis_);
        return;
      }

      // Direction reversal: once notification confirms zero, start the
      // vendor-requested STOP hold before the opposite MoveVelocity.
      if (!rail_stop_hold_active_) {
        rail_stop_hold_active_ = true;
        rail_zero_hold_start_ = this->now();

        RCLCPP_INFO(
          this->get_logger(),
          "[RAIL] Axis%d bNotMoving confirmed; holding STOP for %.0f ms",
          rail_axis_,
          rail_direction_stop_hold_ms_);
        return;
      }

      const double hold_elapsed_ms =
        (this->now() - rail_zero_hold_start_).seconds() * 1000.0;

      if (hold_elapsed_ms < rail_direction_stop_hold_ms_) {
        return;
      }

      rail_target_mps = rail_pending_target_mps_;
      rail_pending_target_mps_ = 0.0;
      rail_stop_sequence_active_ = false;
      rail_waiting_for_stop_ = false;
      rail_stop_hold_active_ = false;
      applied_rail_mps_ = 0.0;

      RCLCPP_INFO(
        this->get_logger(),
        "[RAIL] Axis%d STOP sequence complete; target=%+.3f m/s",
        rail_axis_,
        rail_target_mps);
    }

    // ---------------------------------------------------------------------
    // New explicit STOP request.
    // If the rail is already stopped, nothing needs to be sent.
    // ---------------------------------------------------------------------
    if (std::fabs(rail_target_mps) <= kZeroMps) {
      rail_pending_target_mps_ = 0.0;

      if (rail_physically_stopped) {
        applied_rail_mps_ = 0.0;
        return;
      }

      // A stalled rail may be at zero speed with an active PLC job.
      if (rail_velocity_is_zero && !rail_has_job) {return;}
      rail_stop_sequence_active_ = true;
      rail_waiting_for_stop_ = true;
      rail_stop_tx_sent_ = false;
      rail_stop_retry_wait_active_ = false;
      return;
    }

    // ---------------------------------------------------------------------
    // Direction reversal: Axis3 STOP first. Check real NC direction and the
    // last accepted PLC target, not merely applied_rail_mps_ (which is cleared
    // after a STOP request while the PLC may still own a motion job).
    // ---------------------------------------------------------------------
    if (!read_axis_status(status) || has_any_fault(status)) {
      return; // Never authorize new rail motion on missing/faulted PLC data.
    }
    if (axis_reverse_requires_stop(rail_axis_,
          status[axis_index(rail_axis_)],
          rail_target_mps, rail_sign_))
    {
      rail_pending_target_mps_ = rail_target_mps;
      rail_stop_sequence_active_ = true;
      rail_waiting_for_stop_ = true;
      rail_stop_tx_sent_ = false;
      rail_stop_retry_wait_active_ = false;
      rail_stop_hold_active_ = false;
      return;
    }

    // Same requested speed -> no new ADS command.
    if (
      std::fabs(rail_target_mps - applied_rail_mps_) <=
      rail_command_epsilon_mps_)
    {
      return;
    }

    // Do not overlap a previous rail motion command with a new one.
    if (!previous_command_window_elapsed(rail_axis_, rail_ramp_ms_)) {return;}
    if (stop_attempt_initialized_[static_cast<std::size_t>(rail_axis_)] &&
        (!rail_physically_stopped ||
         !previous_stop_window_elapsed(rail_axis_, rail_ramp_ms_))) {return;}

    // New non-zero Axis3 MoveVelocity.
    const double plc_velocity_mmps =
      rail_target_mps * velocity_scale_ * rail_sign_;

    RCLCPP_DEBUG(
      this->get_logger(),
      "[MOVE TX] Axis%d target=%+.1f mm/s",
      rail_axis_,
      plc_velocity_mmps);

    if (diagnostic_read_only_) return;
    const int32_t result =
      MoveVelocity(
        static_cast<uint32_t>(rail_axis_),
        plc_velocity_mmps,
        rail_ramp_ms_,
        jerk_percent_);

    if (result == NO_ERR_TcMCFUN) {
      applied_rail_mps_ = rail_target_mps;
      note_move_accepted(rail_axis_, rail_target_mps);
      rail_last_move_tx_tp_ = now_tp;
      rail_last_move_tx_initialized_ = true;

      if (log_motion_commands_) {
        RCLCPP_INFO(
          this->get_logger(),
          "[RAIL] Axis%d velocity=%+.1f mm/s (ROS=%+.3f m/s)",
          rail_axis_,
          plc_velocity_mmps,
          rail_target_mps);
      }
    } else {
      if (is_ads_device_busy(result)) {
        enter_safe_stop(std::string("Rail MoveVelocity BUSY Axis") +
          std::to_string(rail_axis_));
        safe_stop_active_ = false;  // Never follow PLC BUSY with StopMove spam.
        RCLCPP_ERROR(this->get_logger(),
          "[RAIL MOVE BUSY NO RETRY] Axis%d; PLC operator intervention required.",
          rail_axis_);
        return;
      }
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[RAIL] MoveVelocity(Axis%d, %+.1f mm/s) failed: 0x%X",
        rail_axis_,
        plc_velocity_mmps,
        static_cast<unsigned int>(result));
    }
  }


  void apply_single_axis_velocity(
    int axis,
    double target_mps,
    double sign,
    double ramp_ms,
    double epsilon_mps,
    double & applied_mps,
    double actual_mps,
    bool & waiting_for_stop,
    double & pending_target_mps,
    const char * label)
  {
    constexpr double kZeroMps = 1.0e-6;
    std::array<AxisStatus, 3> axis_feedback{};
    if (!read_axis_status(axis_feedback) || has_any_fault(axis_feedback)) {
      return;  // Missing feedback is not permission to issue an ADS command.
    }
    const auto & feedback = axis_feedback[axis_index(axis)];
    const bool plc_stopped = axis_is_physically_stopped(feedback, sign) &&
      !feedback.bHasJob && !feedback.bMovingFw && !feedback.bMovingBw;
    const auto axis_idx = static_cast<std::size_t>(axis);
    if (stop_attempt_initialized_[axis_idx]) {
      if (!plc_stopped || !previous_stop_window_elapsed(axis, ramp_ms)) {
        return;  // STOP still in progress: never MOVE or STOP again.
      }
      // Keep the STOP one-shot gate until the next MOVE is accepted.
    }

    // ------------------------------------------------------------
    // A) Direction-change STOP already accepted.
    //    Wait for physical stop before issuing opposite direction.
    // ------------------------------------------------------------
    if (waiting_for_stop) {
      pending_target_mps = target_mps;

      if (!plc_stopped) {
        return;  // Do not reverse before PLC bNotMoving and speed agree.
      }

      // Physical stop confirmed.
      applied_mps = 0.0;

      // User/watchdog changed the desired target to STOP while decelerating.
      if (std::fabs(target_mps) <= kZeroMps) {
        pending_target_mps = 0.0;
        waiting_for_stop = false;
        return;
      }

      waiting_for_stop = false;
      target_mps = pending_target_mps;
      pending_target_mps = 0.0;
    }

    // ------------------------------------------------------------
    // B) STOP has highest priority.
    //
    // Important difference from the old code:
    // even if applied_mps was already set to 0, if ACTUAL velocity is still
    // non-zero we continue trying StopMove. This prevents the observed state:
    // CMD=0 while Axis2 continues moving.
    // ------------------------------------------------------------
    if (std::fabs(target_mps) <= kZeroMps) {
      pending_target_mps = 0.0;

      if (plc_stopped) {
        waiting_for_stop = false;
        applied_mps = 0.0;
        return;
      }
      // Never ignore a STOP because a mechanically blocked motor reads zero.
      // request_axis_stop() verifies HasJob, FB status and one-shot limits.
      const auto stop_result =
        request_axis_stop(axis, ramp_ms, label);

      if (stop_result == StopRequestResult::ACCEPTED) {
        applied_mps = 0.0;
        waiting_for_stop = false;
      }
      // BUSY / RATE_LIMITED / FAILED:
      // keep applied_mps as-is so the next control cycles continue to know
      // a real motion command may still be active in the PLC.

      return;
    }

    if (stop_attempt_initialized_[axis_idx] && !plc_stopped) {
      return;
    }

    // ------------------------------------------------------------
    // C) Same requested speed -> no new ADS command.
    // ------------------------------------------------------------
    if (std::fabs(target_mps - applied_mps) <= epsilon_mps) {
      return;
    }

    // ------------------------------------------------------------
    // D) Direction change -> StopMove first.
    //    If StopMove is BUSY, do NOT zero applied_mps and do NOT send the
    //    opposite direction. Retry STOP later.
    // ------------------------------------------------------------
    if (
      !plc_stopped &&
      axis_reverse_requires_stop(axis, feedback, target_mps, sign))
    {
      const auto stop_result =
        request_axis_stop(axis, ramp_ms, "direction change");

      if (stop_result != StopRequestResult::ACCEPTED) {
        return;
      }

      pending_target_mps = target_mps;
      waiting_for_stop = true;
      applied_mps = 0.0;

      if (log_motion_commands_) {
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] Axis%d direction change: STOP accepted, waiting for physical zero",
          label,
          axis);
      }

      return;
    }

    // ------------------------------------------------------------
    // E) New non-zero speed.
    //    Coalesce high-rate Nav2 updates. The latest requested target remains
    //    in requested_linear/angular and will be sent on a later control cycle.
    //    Zero/STOP and direction-change STOP paths above bypass this limiter.
    // ------------------------------------------------------------
    if (axis >= 1 && axis <= 3 && wheel_move_command_min_interval_ms_ > 0.0) {
      const auto now_tp = std::chrono::steady_clock::now();
      const std::size_t idx = static_cast<std::size_t>(axis);
      if (move_attempt_initialized_[idx]) {
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
          now_tp - last_move_attempt_tp_[idx]).count();
        if (elapsed_ms < wheel_move_command_min_interval_ms_) {
          return;
        }
      }
      move_attempt_initialized_[idx] = true;
      last_move_attempt_tp_[idx] = now_tp;
    }

    // PLC methods must not overlap. A ramp+margin software gate is used
    // until the vendor exposes explicit MoveVelocity_Status completion bits.
    if (!previous_command_window_elapsed(axis, ramp_ms)) {return;}
    // Re-check the safety latch immediately before the vendor API call.
    if (fault_latched_ || safe_stop_latched_ || !motion_ready_) {
      return;
    }

    // Final PLC direction interlock immediately before the ADS write.
    // If a STOP was requested on a prior cycle, allow a new MOVE only after
    // the STOP FB has released and the motor is physically stopped.
    std::array<AxisStatus, 3> final_fb{};
    if (!read_axis_status(final_fb) || has_any_fault(final_fb)) return;
    const auto & final_axis = final_fb[axis_index(axis)];
    if (axis_reverse_requires_stop(axis, final_axis, target_mps, sign)) {
      (void)request_axis_stop(axis, ramp_ms, "final TX direction guard");
      return;
    }

    const double plc_velocity_mmps =
      target_mps *
      velocity_scale_ *
      sign;

    // V6 final TX safety gate.
    // Re-check immediately before the vendor ADS call. A fault/latch can
    // be raised after the upper-level command path was entered.
    if (fault_latched_ || safe_stop_latched_ || !motion_ready_ || wheel_stop_pending_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[MOVE BLOCKED] Axis%d target=%+.3f m/s fault=%d latch=%d ready=%d stop_pending=%d",
        axis, target_mps,
        fault_latched_ ? 1 : 0,
        safe_stop_latched_ ? 1 : 0,
        motion_ready_ ? 1 : 0,
        wheel_stop_pending_ ? 1 : 0);
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[MOVE TX] Axis%d target=%+.1f mm/s",
      axis, plc_velocity_mmps);

    if (diagnostic_read_only_) return;
    const int32_t result =
      MoveVelocity(
        static_cast<uint32_t>(axis),
        plc_velocity_mmps,
        ramp_ms,
        jerk_percent_);

    if (result == NO_ERR_TcMCFUN) {
      applied_mps = target_mps;
      note_move_accepted(axis, target_mps);

      if (log_motion_commands_) {
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] Axis%d velocity=%+.1f mm/s (ROS=%+.3f m/s)",
          label,
          axis,
          plc_velocity_mmps,
          target_mps);
      }
    } else {
      if (is_ads_device_busy(result)) {
        enter_safe_stop(std::string("MoveVelocity BUSY Axis") + std::to_string(axis));
        safe_stop_active_ = false;  // No automatic StopMove on a BUSY drive.
        RCLCPP_ERROR(this->get_logger(),
          "[MOVE BUSY NO RETRY] Axis%d. ROS motion blocked; "
          "PLC safety stop/operator intervention required.", axis);
        return;
      }
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[%s] MoveVelocity(Axis%d, %+.1f mm/s) failed: 0x%X",
        label,
        axis,
        plc_velocity_mmps,
        static_cast<unsigned int>(result));
    }
  }

  // ============================================================
  // Stop
  // ============================================================
  void stop_all_axes(const std::string & reason)
  {
    clear_motion_requests();

    std::array<AxisStatus, 3> status{};
    const bool have_status = read_axis_status(status);

    const auto request_if_needed =
      [&](int axis, double ramp_ms, double sign, double & applied_mps)
      {
        bool stopped = false;

        if (have_status) {
          const auto & s =
            status[axis_index(axis)];
          stopped = axis_is_physically_stopped(s, sign) &&
            !s.bHasJob && !s.bMovingFw && !s.bMovingBw;
        }

        if (stopped) {
          applied_mps = 0.0;
          return;
        }

        const auto result =
          request_axis_stop(axis, ramp_ms, reason.c_str());

        if (result == StopRequestResult::ACCEPTED) {
          applied_mps = 0.0;
        }
        // BUSY: leave applied_mps unchanged. A later control/status cycle
        // must retry STOP. Most importantly, do not PowerDisable yet.
      };

    request_if_needed(
      left_wheel_axis_, wheel_ramp_ms_, left_wheel_sign_, applied_left_mps_);
    request_if_needed(
      right_wheel_axis_, wheel_ramp_ms_, right_wheel_sign_, applied_right_mps_);
    request_if_needed(
      rail_axis_, rail_ramp_ms_, rail_sign_, applied_rail_mps_);
  }

  // ============================================================
  // Feedback publishers
  // ============================================================
  void publish_wheel_feedback(double left_mps, double right_mps)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
      static_cast<float>(left_mps),
      static_cast<float>(right_mps)};
    wheel_speed_feedback_pub_->publish(msg);
  }

  void publish_rail_feedback(double rail_mps)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {static_cast<float>(rail_mps)};
    rail_speed_feedback_pub_->publish(msg);
  }

  void integrate_and_publish_wheel_odom(double left_mps, double right_mps)
  {
    const auto now = this->now();
    double dt = (now - last_feedback_time_).seconds();

    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.0;
    }

    last_feedback_time_ = now;

    // Called only with successful, finite PLC feedback. A motion-inhibit latch
    // must not falsely report zero velocity while the machine is still coasting.
    // PLC wheel feedback is not independent ground-truth localization: slip and
    // invalid encoder data still require fusion/diagnostic handling upstream.
    const double linear_mps = (left_mps + right_mps) / 2.0;
    const double angular_rps = (right_mps - left_mps) / wheel_separation_m_;

    if (dt > 0.0) {
      const double delta_theta = angular_rps * dt;
      const double theta_mid = odom_theta_ + delta_theta / 2.0;

      odom_x_ += linear_mps * std::cos(theta_mid) * dt;
      odom_y_ += linear_mps * std::sin(theta_mid) * dt;
      odom_theta_ += delta_theta;
      odom_theta_ = std::atan2(std::sin(odom_theta_), std::cos(odom_theta_));
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "agv_base_footprint";

    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.position.z = 0.0;

    const double half_yaw = odom_theta_ / 2.0;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);

    odom.twist.twist.linear.x = linear_mps;
    odom.twist.twist.angular.z = angular_rps;

    odom.pose.covariance[0] = 0.10;
    odom.pose.covariance[7] = 0.10;
    odom.pose.covariance[35] = 0.20;
    odom.twist.covariance[0] = 0.02;
    odom.twist.covariance[35] = 0.05;

    wheel_odom_pub_->publish(odom);
  }

  void publish_joint_states(double left_mps, double right_mps)
  {
    const auto now = this->now();
    double dt = (now - last_joint_time_).seconds();

    if (last_joint_time_.nanoseconds() == 0) {
      dt = 0.0;
    }

    last_joint_time_ = now;

    const double radius_m = wheel_diameter_m_ / 2.0;
    const double left_rad_s = left_mps / radius_m;
    const double right_rad_s = right_mps / radius_m;

    if (dt > 0.0 && dt < 1.0) {
      left_joint_position_rad_ += left_rad_s * dt;
      right_joint_position_rad_ += right_rad_s * dt;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now;
    msg.name = {"wheel_left_joint", "wheel_right_joint"};
    msg.position = {left_joint_position_rad_, right_joint_position_rad_};
    msg.velocity = {left_rad_s, right_rad_s};
    joint_state_pub_->publish(msg);
  }

  // ============================================================
  // Mock mode
  // ============================================================
  void apply_mock_commands(double linear_mps, double angular_rps, double rail_mps)
  {
    applied_left_mps_ =
      linear_mps - angular_rps * wheel_separation_m_ / 2.0;
    applied_right_mps_ =
      linear_mps + angular_rps * wheel_separation_m_ / 2.0;
    applied_rail_mps_ = rail_mps;
  }

  void publish_mock_feedback()
  {
    publish_wheel_feedback(applied_left_mps_, applied_right_mps_);
    publish_rail_feedback(applied_rail_mps_);
    integrate_and_publish_wheel_odom(applied_left_mps_, applied_right_mps_);
    publish_joint_states(applied_left_mps_, applied_right_mps_);
  }

  // ============================================================
  // Status
  // ============================================================
  void publish_status()
  {
    std_msgs::msg::String msg;

    if (mock_mode_) {
      msg.data = "AGV ADS Driver: MOCK MODE";
    } else if (ads_reconnect_transport_needed_) {
      msg.data = "AGV ADS Driver: ADS DISCONNECTED / AUTO RECONNECT PENDING / MOTION LOCKED";
    } else if (ads_recovery_required_) {
      msg.data = "AGV ADS Driver: ADS RECONNECTED / WAITING OPERATOR APPROVAL / MOTION LOCKED";
    } else if (!ads_connected_) {
      msg.data = "AGV ADS Driver: ADS DISCONNECTED";
    } else if (reset_rearm_pending_) {
      msg.data = reset_waiting_for_approval_ && !power_restore_approved_ ?
        "AGV ADS Driver: RESET COMPLETE / WAITING OPERATOR POWER APPROVAL" :
        "AGV ADS Driver: RESET REQUESTED / PLC RE-ARM PENDING";
    } else if (safe_stop_latched_) {
      msg.data = "AGV ADS Driver: SAFE STOP LATCHED / " + safe_stop_reason_;
    } else if (fault_latched_) {
      msg.data = "AGV ADS Driver: AXIS FAULT LATCHED";
    } else if (diagnostic_read_only_) {
      msg.data = "AGV ADS Driver: DIAGNOSTIC READ-ONLY / NO ADS MOTION WRITES";
    } else if (!motion_ready_) {
      msg.data = "AGV ADS Driver: CONNECTED / MOTION NOT READY";
    } else {
      msg.data =
        "AGV ADS Driver: READY / rail_state=" + rail_state_ +
        " / wheels=" + std::string(wheel_axes_enabled_ ? "ENABLE" : "DISABLE") +
        " / rail=" + std::string(rail_axis_enabled_ ? "ENABLE" : "DISABLE") +
        " / dual_opt_in=" + std::string(allow_simultaneous_drive_ ? "ON" : "OFF");
    }

    status_pub_->publish(msg);
  }

private:
  bool mock_mode_{true};
  bool ads_connected_{false};
  bool ads_auto_reconnect_{true};
  double ads_reconnect_interval_sec_{5.0};
  int ads_disconnect_failure_count_{3};
  int ads_failure_count_{0};
  int ads_fb_failure_count_{0};
  bool ads_recovery_required_{false};
  bool ads_reconnect_transport_needed_{false};
  std::chrono::steady_clock::time_point last_ads_reconnect_tp_{};
  std::chrono::steady_clock::time_point last_ads_health_tp_{};
  bool motion_ready_{false};
  bool fault_latched_{false};
  bool fault_stop_active_{false};

  // Fail-safe latch: once entered, no MoveVelocity command is accepted until
  // /agv/reset_faults completes successfully. This is a ROS-side safety layer;
  // a PLC watchdog / hardware E-stop is still required for process/host failure.
  bool safe_stop_latched_{false};
  bool safe_stop_active_{false};
  bool wheel_motion_armed_{false};
  bool rail_motion_armed_{false};
  std::string safe_stop_reason_;

  bool shutting_down_{false};
  bool reset_rearm_pending_{false};
  bool reset_waiting_for_nc_release_{false};
  std::chrono::steady_clock::time_point reset_waiting_nc_started_{};
  static constexpr double kNcReleaseWaitTimeoutSec = 30.0;
  bool reset_power_restore_sent_{false};
  bool reset_waiting_for_approval_{false};
  bool power_restore_approved_{false};
  bool require_power_restore_approval_{true};
  double reset_standstill_verify_sec_{3.0};
  double reset_recovery_timeout_sec_{5.0};
  bool reset_standstill_verifying_{false};
  std::chrono::steady_clock::time_point reset_standstill_started_{};
  std::array<bool, 4> fallback_disable_attempted_{{false, false, false, false}};
  std::array<bool, 4> fault_disable_attempted_{{false, false, false, false}};
  bool stale_recovery_pending_{false};
  bool stale_recovery_stops_sent_{false};
  uint32_t stale_recovery_mask_{0};
  std::chrono::steady_clock::time_point stale_recovery_started_{};
  std::chrono::steady_clock::time_point reset_rearm_started_{};

  // Legacy power interlock remains default. Only an explicit experimental
  // opt-in allows independent wheel and rail enables/motion commands. The
  // hardware/PLC safety protection must be independently validated.
  std::string rail_state_{"OUT_RAIL"};
  bool wheel_axes_enabled_{false};
  bool rail_axis_enabled_{false};
  PowerTransitionStage power_transition_stage_{PowerTransitionStage::NONE};
  bool power_stage_command_sent_{false};

  int left_wheel_axis_{1};
  int right_wheel_axis_{2};
  int rail_axis_{3};

  double left_wheel_sign_{1.0};
  double right_wheel_sign_{1.0};
  double rail_sign_{1.0};

  double wheel_separation_m_{0.930};
  double wheel_diameter_m_{0.2032};
  double velocity_scale_{1000.0};

  double wheel_ramp_ms_{1000.0};
  double rail_ramp_ms_{1000.0};
  double rail_direction_stop_hold_ms_{200.0};
  double jerk_percent_{100.0};

  double cmd_vel_timeout_sec_{10.0};
  double rail_cmd_timeout_sec_{10.0};
  bool auto_reset_axis_fault_{false};  // OFF until PLC/drive reset safety validated.
  bool disable_on_stop_failure_{true};
  std::array<bool, 4> fault_reset_attempted_{{false, false, false, false}};

  double wheel_command_epsilon_mps_{0.005};

  // IMU-assisted steering is independent from motor velocity balancing.
  bool imu_correction_enable_{true};
  double imu_timeout_sec_{0.5};
  double imu_heading_kp_{0.30};
  double imu_heading_kd_{0.1};
  double imu_turn_kp_{0.3};
  double imu_max_angular_correction_rps_{0.05};
  double imu_max_correction_step_rps_{0.015};
  double imu_correction_interval_sec_{2.0};
  double imu_straight_angular_threshold_rps_{0.03};
  double imu_min_wheel_speed_mps_{0.08};
  double imu_yaw_sign_{-1.0};
  bool imu_valid_{false};
  bool imu_straight_active_{false};
  bool imu_eval_initialized_{false};
  double imu_yaw_rad_{0.0};
  double imu_yaw_rate_rps_{0.0};
  double imu_target_yaw_rad_{0.0};
  double imu_angular_correction_rps_{0.0};
  std::chrono::steady_clock::time_point imu_last_rx_tp_{};
  std::chrono::steady_clock::time_point imu_last_eval_tp_{};

  // Axis1/2 actual-velocity balance controller (straight, pivot, curves).
  bool wheel_balance_enable_{true};
  double wheel_balance_kp_{0.20};
  double wheel_balance_max_correction_mps_{0.010};
  double wheel_balance_deadband_mps_{0.005};
  double wheel_balance_min_speed_mps_{0.050};
  double wheel_balance_angular_threshold_rps_{0.030};
  double wheel_balance_filter_alpha_{0.15};
  double wheel_balance_settle_sec_{1.50};
  double wheel_balance_stable_hold_sec_{1.0};
  double wheel_balance_release_rate_mps2_{0.005};
  double wheel_balance_reverse_hold_sec_{0.25};
  bool wheel_balance_stable_tracking_{false};
  bool wheel_balance_release_active_{false};
  bool wheel_balance_reverse_tracking_{false};
  bool wheel_balance_tick_initialized_{false};
  std::chrono::steady_clock::time_point wheel_balance_stable_since_{};
  std::chrono::steady_clock::time_point wheel_balance_reverse_since_{};
  std::chrono::steady_clock::time_point wheel_balance_last_tick_tp_{};
  bool wheel_balance_reference_initialized_{false};
  double wheel_balance_reference_left_mps_{0.0};
  double wheel_balance_reference_right_mps_{0.0};
  bool wheel_balance_eval_initialized_{false};
  std::chrono::steady_clock::time_point wheel_balance_last_eval_tp_{};
  bool wheel_balance_initialized_{false};
  double wheel_balance_error_filtered_mps_{0.0};
  double wheel_balance_correction_mps_{0.0};

  double wheel_move_command_min_interval_ms_{100.0};
  double ads_command_settle_extra_ms_{150.0};
  double stop_suppress_velocity_mps_{0.002};
  double axis_notification_max_age_sec_{1.5};
  double wheel_stop_pending_timeout_sec_{4.0};
  double rail_command_epsilon_mps_{0.001};

  bool power_enable_on_start_{true};
  bool diagnostic_read_only_{false};
  double startup_power_transition_timeout_sec_{5.0};
  bool power_disable_on_shutdown_{true};
  double shutdown_poweroff_timeout_sec_{7.0};
  bool allow_simultaneous_drive_{false};
  double dual_wheel_axis_max_mps_{0.05};
  double dual_rail_max_mps_{0.10};
  bool log_motion_commands_{false};

  bool log_axis_monitor_{true};
  double axis_monitor_period_sec_{0.5};
  double diagnostic_ads_poll_period_sec_{0.5};
  double ads_slow_read_warn_ms_{100.0};
  bool last_diagnostic_poll_initialized_{false};
  std::chrono::steady_clock::time_point last_diagnostic_poll_tp_{};

  // Environmental sensor over Beckhoff ADS.
  bool enable_environment_sensor_{true};
  double environment_publish_period_sec_{1.0};
  bool log_environment_sensor_{true};
  std::atomic<bool> environment_stop_{false};
  std::thread environment_worker_{};
  std::mutex environment_mutex_{};
  SensorInfo environment_sensor_data_{};
  bool environment_sample_valid_{false};
  std::chrono::steady_clock::time_point environment_sample_time_{};
  uint64_t environment_sample_sequence_{0};
  uint64_t environment_published_sequence_{0};

  double stop_retry_interval_ms_{1000.0};
  double stop_wait_timeout_sec_{4.0};
  double stopped_velocity_threshold_mps_{0.01};

  // Index 1..3 maps directly to Beckhoff Axis1..Axis3.
  std::array<std::chrono::steady_clock::time_point, 4> last_stop_attempt_tp_{};
  std::array<bool, 4> stop_attempt_initialized_{{false, false, false, false}};
  // Per axis: last accepted MoveVelocity time (for command overlap prevention).
  std::array<std::chrono::steady_clock::time_point, 4> last_accepted_move_tp_{};
  std::array<bool, 4> last_accepted_move_initialized_{{false, false, false, false}};
  std::array<double, 4> last_accepted_target_mps_{{0.0, 0.0, 0.0, 0.0}};
  std::array<bool, 4> move_feedback_stable_initialized_{{false, false, false, false}};
  std::array<std::chrono::steady_clock::time_point, 4> move_feedback_stable_since_{};
  std::array<std::chrono::steady_clock::time_point, 4> last_stop_tx_tp_{};
  std::array<bool, 4> last_stop_tx_initialized_{{false, false, false, false}};
  // Inactive MC_Stop FB may report Busy=0, Done=0.  Track continuous
  // idle+standstill feedback before allowing new motion for that axis.
  std::array<bool, 4> stop_idle_confirm_initialized_{{false, false, false, false}};
  std::array<std::chrono::steady_clock::time_point, 4> stop_idle_confirm_since_{};
  std::array<std::chrono::steady_clock::time_point, 4> last_move_attempt_tp_{};
  std::array<bool, 4> move_attempt_initialized_{{false, false, false, false}};

  bool wheel_stop_pending_{false};
  bool wheel_direction_change_active_{false};
  bool wheel_pair_stable_initialized_{false};
  std::chrono::steady_clock::time_point wheel_pair_stable_since_{};
  double wheel_direction_pending_left_mps_{0.0};
  double wheel_direction_pending_right_mps_{0.0};
  std::chrono::steady_clock::time_point wheel_stop_pending_since_{};

  // No modification to TcMC/TcMCApi vendor sources: direct status reader.
  PlcFbStatusReader plc_fb_reader_{};
  std::array<AxisStatus, 3> axis_snapshot_{};
  bool axis_snapshot_valid_{false};
  std::chrono::steady_clock::time_point axis_snapshot_time_{};

  bool axis_state_initialized_{false};
  bool prev_left_ready_{false};
  bool prev_right_ready_{false};
  bool prev_rail_ready_{false};
  bool prev_left_error_{false};
  bool prev_right_error_{false};
  bool prev_rail_error_{false};
  uint32_t prev_left_error_code_{0};
  uint32_t prev_right_error_code_{0};
  uint32_t prev_rail_error_code_{0};

  double requested_linear_x_mps_{0.0};
  double requested_angular_z_rps_{0.0};
  double requested_rail_speed_mps_{0.0};

  double applied_left_mps_{0.0};
  double applied_right_mps_{0.0};
  double applied_rail_mps_{0.0};

  // Latest measured velocities from Beckhoff feedback.
  double actual_left_mps_{0.0};
  double actual_right_mps_{0.0};
  double actual_rail_mps_{0.0};

  // Direction-change interlock state.
  bool left_waiting_for_stop_{false};
  bool right_waiting_for_stop_{false};
  bool rail_waiting_for_stop_{false};
  bool rail_stop_hold_active_{false};

  // Axis3 STOP state machine.
  // Stop completion is driven by AxisStatus Notification (bNotMoving), not
  // synchronous ADS polling and not periodic StopMove retransmission.
  bool rail_stop_sequence_active_{false};
  bool rail_stop_tx_sent_{false};
  bool rail_stop_retry_wait_active_{false};
  bool rail_last_move_tx_initialized_{false};

  std::chrono::steady_clock::time_point rail_last_move_tx_tp_{};
  std::chrono::steady_clock::time_point rail_stop_tx_tp_{};
  std::chrono::steady_clock::time_point rail_stop_retry_wait_start_tp_{};

  double left_pending_target_mps_{0.0};
  double right_pending_target_mps_{0.0};
  double rail_pending_target_mps_{0.0};

  // One-shot watchdog logging flags.
  bool wheel_watchdog_triggered_{false};
  bool rail_watchdog_triggered_{false};

  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_rail_cmd_time_;
  std::chrono::steady_clock::time_point last_cmd_vel_steady_{};
  std::chrono::steady_clock::time_point last_rail_cmd_steady_{};
  rclcpp::Time last_feedback_time_;
  rclcpp::Time last_axis_monitor_time_;
  rclcpp::Time last_environment_publish_time_;
  rclcpp::Time rail_zero_hold_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_theta_{0.0};

  double left_joint_position_rad_{0.0};
  double right_joint_position_rad_{0.0};

  std::size_t status_counter_{0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rail_cmd_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rail_state_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    wheel_speed_feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    rail_speed_feedback_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr axis_diagnostics_pub_;
  bool axis_diag_published_{false};
  std::chrono::steady_clock::time_point axis_diag_last_tp_{};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    environment_data_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    environment_solar_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    environment_temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    environment_humidity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    environment_co2_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_diagnostics_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_faults_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr approve_power_restore_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr approve_ads_recovery_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_stale_jobs_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr wheel_power_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr rail_power_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  // TwinCAT Runtime STOP/RUN closes the ADS TCP connection asynchronously.
  // On Linux, a write to that closed socket can deliver SIGPIPE and terminate
  // this process with exit code -13 before ADS reconnect can run. Ignore the
  // process-wide signal; the ADS API must still report the failed I/O normally.
  // Do not interpret SIGPIPE suppression as proof of PLC/motor safety.
  if (std::signal(SIGPIPE, SIG_IGN) == SIG_ERR) {
    std::fprintf(stderr, "[AGV] Failed to install SIGPIPE handler\n");
    return 1;
  }

  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<AgvAdsDriver>();
    rclcpp::spin(node);
  }
  catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("agv_ads_driver"),
      "Fatal error: %s",
      e.what());
  }

  rclcpp::shutdown();
  return 0;
}