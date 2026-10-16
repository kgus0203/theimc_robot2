#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

// Vendor-provided Beckhoff wrapper API.
#include "TcMCApi.h"

using namespace std::chrono_literals;

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
    this->declare_parameter<double>("wheel_separation_m", 1.0);

    // Vendor confirmed driving wheel diameter = 206 mm.
    this->declare_parameter<double>("wheel_diameter_m", 0.206);

    // Vendor confirmed API units:
    // MoveVelocity velocity = mm/s
    // AxisStatus.lActVelocity = mm/s
    // AxisStatus.lActPosition = mm
    this->declare_parameter<double>("velocity_scale", 1000.0);  // m/s -> mm/s

    // Vendor confirmed:
    // ramp = milliseconds
    // jerk = 0..100 (% S-curve amount)
    this->declare_parameter<double>("wheel_ramp_ms", 700.0);
    this->declare_parameter<double>("rail_ramp_ms", 700.0);
    // Rail Axis3 requires an explicit STOP before reversing direction.
    // After feedback reaches near-zero, keep STOP state for this additional hold time.
    this->declare_parameter<double>("rail_direction_stop_hold_ms", 200.0);
    this->declare_parameter<double>("jerk_percent", 100.0);

    this->declare_parameter<double>("cmd_vel_timeout_sec", 10.0);
    this->declare_parameter<double>("rail_cmd_timeout_sec", 10.0);

    // Avoid repeatedly sending nearly-identical MoveVelocity requests.
    this->declare_parameter<double>("wheel_command_epsilon_mps", 0.005);
    // Limit Beckhoff MoveVelocity ADS calls. Nav2 may publish at 20+ Hz, but
    // the PLC motion method must not be called for every small velocity update.
    // STOP commands are NOT rate-limited by this setting.
    this->declare_parameter<double>("wheel_move_command_min_interval_ms", 100.0);
    // BUSY stop interlock: block new wheel motion until both wheels stop.
    this->declare_parameter<double>("wheel_stop_pending_timeout_sec", 4.0);
    this->declare_parameter<double>("rail_command_epsilon_mps", 0.001);

    // Vendor says PowerEnable is required once before motion.
    this->declare_parameter<bool>("power_enable_on_start", true);
    this->declare_parameter<bool>("power_disable_on_shutdown", true);

    // Normal MoveVelocity/StopMove command logs can become very noisy
    // because Nav2 may change /cmd_vel frequently. Keep them off by default.
    // Errors and important state transitions are still logged.
    this->declare_parameter<bool>("log_motion_commands", false);

    // Axis diagnostic monitor.
    // Prints command/actual speed, Ready, Error, ErrorCode for Axis1/2/3.
    // This does not change the motion behavior.
    this->declare_parameter<bool>("log_axis_monitor", false);
    this->declare_parameter<double>("axis_monitor_period_sec", 0.5);

    // Safe STOP handling.
    //
    // Vendor sample sequence is:
    //   StopMove() -> wait until AxisStatus.bNotMoving -> PowerDisable()
    //
    // Axis2 has returned 0x98110708 (ADSERR_DEVICE_BUSY) when StopMove is
    // requested. Never PowerDisable a wheel that is still moving. When BUSY
    // occurs, keep power enabled, inhibit new motion, and retry STOP at a
    // bounded interval.
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

    wheel_command_epsilon_mps_ =
      std::fabs(this->get_parameter("wheel_command_epsilon_mps").as_double());
    wheel_move_command_min_interval_ms_ =
      this->get_parameter("wheel_move_command_min_interval_ms").as_double();
    wheel_stop_pending_timeout_sec_ =
      this->get_parameter("wheel_stop_pending_timeout_sec").as_double();
    rail_command_epsilon_mps_ =
      std::fabs(this->get_parameter("rail_command_epsilon_mps").as_double());

    power_enable_on_start_ = this->get_parameter("power_enable_on_start").as_bool();
    power_disable_on_shutdown_ = this->get_parameter("power_disable_on_shutdown").as_bool();
    log_motion_commands_ = this->get_parameter("log_motion_commands").as_bool();

    log_axis_monitor_ = this->get_parameter("log_axis_monitor").as_bool();
    axis_monitor_period_sec_ =
      this->get_parameter("axis_monitor_period_sec").as_double();

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

    validate_parameters();

    // ============================================================
    // ROS subscriptions
    // ============================================================
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&AgvAdsDriver::cmd_vel_callback, this, std::placeholders::_1));

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

    reset_faults_srv_ =
      this->create_service<std_srvs::srv::Trigger>(
      "/agv/reset_faults",
      std::bind(
        &AgvAdsDriver::reset_faults_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

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
    last_feedback_time_ = this->now();
    last_axis_monitor_time_ = this->now();
    last_environment_publish_time_ = this->now();
    environment_notification_start_time_ = this->now();

    // ============================================================
    // Beckhoff connection / initial PowerEnable
    // ============================================================
    if (!mock_mode_) {
      connect_and_initialize();
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

    RCLCPP_WARN(
      this->get_logger(),
      "Axis1/2 left-right assignment and direction signs depend on assembly. "
      "Verify at low speed before normal operation.");

    RCLCPP_WARN(
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

    if (timer_) {
      timer_->cancel();
    }

    if (!mock_mode_ && ads_connected_) {
      clear_motion_requests();

      const bool stopped =
        wait_until_all_axes_stopped(stop_wait_timeout_sec_, "node shutdown");

      if (power_disable_on_shutdown_) {
        if (stopped) {
          power_disable_all_axes();
        } else {
          // IMPORTANT:
          // Do not remove Controller Enable while an axis is still moving.
          // That can create a controller-enable fault (for example 0x4260).
          RCLCPP_ERROR(
            this->get_logger(),
            "[SAFE STOP] Shutdown timeout: one or more axes are still moving. "
            "Skipping PowerDisable to avoid disabling a moving axis. "
            "Use PLC/drive watchdog or E-stop for final hardware stop.");
        }
      }

      axis_status_notification_registered_ = false;
      Disconnect();
      ads_connected_ = false;

      RCLCPP_INFO(this->get_logger(), "Beckhoff ADS disconnected.");
    }
  }

private:
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

    if (!std::isfinite(axis_monitor_period_sec_) ||
        axis_monitor_period_sec_ <= 0.0)
    {
      throw std::runtime_error("axis_monitor_period_sec must be > 0");
    }

    if (!std::isfinite(wheel_move_command_min_interval_ms_) ||
        wheel_move_command_min_interval_ms_ < 0.0)
    {
      throw std::runtime_error("wheel_move_command_min_interval_ms must be >= 0");
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
  }

  void enter_safe_stop(const std::string & reason)
  {
    clear_motion_requests();
    motion_ready_ = false;
    safe_stop_active_ = true;

    if (!safe_stop_latched_) {
      safe_stop_latched_ = true;
      safe_stop_reason_ = reason;
      stop_attempt_initialized_.fill(false);
      RCLCPP_ERROR(
        this->get_logger(),
        "[SAFE STOP LATCH] %s. All new motion commands are inhibited until "
        "/agv/reset_faults succeeds.",
        reason.c_str());
    }
  }

  void process_safe_stop(const std::array<AxisStatus, 3> & status)
  {
    if (!safe_stop_latched_ || !safe_stop_active_) {
      return;
    }

    const auto & left = status[static_cast<std::size_t>(left_wheel_axis_ - 1)];
    const auto & right = status[static_cast<std::size_t>(right_wheel_axis_ - 1)];
    const auto & rail = status[static_cast<std::size_t>(rail_axis_ - 1)];

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

  bool stop_retry_due(int axis)
  {
    if (axis < 1 || axis > 3) {
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const std::size_t idx = static_cast<std::size_t>(axis);

    if (!stop_attempt_initialized_[idx]) {
      stop_attempt_initialized_[idx] = true;
      last_stop_attempt_tp_[idx] = now;
      return true;
    }

    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
        now - last_stop_attempt_tp_[idx]).count();

    if (elapsed_ms < stop_retry_interval_ms_) {
      return false;
    }

    last_stop_attempt_tp_[idx] = now;
    return true;
  }

  StopRequestResult request_axis_stop(
    int axis,
    double ramp_ms,
    const char * context)
  {
    if (!stop_retry_due(axis)) {
      return StopRequestResult::RATE_LIMITED;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 250,
      "[STOP TX] Axis%d context=%s",
      axis, context);

    const int32_t result =
      StopMove(
        static_cast<uint32_t>(axis),
        ramp_ms,
        jerk_percent_);

    if (result == NO_ERR_TcMCFUN) {
      if (log_motion_commands_) {
        RCLCPP_INFO(
          this->get_logger(),
          "[SAFE STOP] StopMove accepted: Axis%d (%s)",
          axis,
          context);
      }
      return StopRequestResult::ACCEPTED;
    }

    if (is_ads_device_busy(result)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[SAFE STOP] Axis%d StopMove BUSY (0x98110708) during %s. "
        "Keeping Controller Enable ON and retrying every %.0f ms.",
        axis,
        context,
        stop_retry_interval_ms_);
      // BUSY may be transient, but a wheel may still be executing its last
      // MoveVelocity. Block ALL new wheel MoveVelocity until both wheels have
      // physically stopped. This temporary state clears automatically.
      if (
        !shutting_down_ && !safe_stop_latched_ && !fault_latched_ &&
        (axis == left_wheel_axis_ || axis == right_wheel_axis_))
      {
        if (!wheel_stop_pending_) {
          wheel_stop_pending_ = true;
          wheel_stop_pending_since_ = std::chrono::steady_clock::now();
          RCLCPP_WARN(
            this->get_logger(),
            "[STOP_PENDING] Wheel StopMove returned BUSY. "
            "Blocking new wheel MoveVelocity until both wheels stop.");
        }
      }
      return StopRequestResult::BUSY;
    }

    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "[SAFE STOP] Axis%d StopMove failed during %s: 0x%X",
      axis,
      context,
      static_cast<unsigned int>(result));
    // A single StopMove transport/API failure must not permanently lock normal
    // navigation.  The caller keeps the zero target and retries STOP.  Actual
    // axis faults are latched separately in process_axis_status().
    return StopRequestResult::FAILED;
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
        status[static_cast<std::size_t>(left_wheel_axis_ - 1)],
        left_wheel_sign_) &&
      axis_is_physically_stopped(
        status[static_cast<std::size_t>(right_wheel_axis_ - 1)],
        right_wheel_sign_) &&
      axis_is_physically_stopped(
        status[static_cast<std::size_t>(rail_axis_ - 1)],
        rail_sign_);
  }

  bool wait_until_all_axes_stopped(
    double timeout_sec,
    const std::string & reason)
  {
    clear_motion_requests();
    stop_attempt_initialized_.fill(false);

    const auto start = std::chrono::steady_clock::now();

    while (true) {
      std::array<AxisStatus, 3> status{};

      if (read_axis_status(status)) {
        const auto & left =
          status[static_cast<std::size_t>(left_wheel_axis_ - 1)];
        const auto & right =
          status[static_cast<std::size_t>(right_wheel_axis_ - 1)];
        const auto & rail =
          status[static_cast<std::size_t>(rail_axis_ - 1)];

        const bool left_stopped =
          axis_is_physically_stopped(left, left_wheel_sign_);
        const bool right_stopped =
          axis_is_physically_stopped(right, right_wheel_sign_);
        const bool rail_stopped =
          axis_is_physically_stopped(rail, rail_sign_);

        if (left_stopped && right_stopped && rail_stopped) {
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
            final_status[static_cast<std::size_t>(left_wheel_axis_ - 1)];
          const auto & right =
            final_status[static_cast<std::size_t>(right_wheel_axis_ - 1)];
          const auto & rail =
            final_status[static_cast<std::size_t>(rail_axis_ - 1)];

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
    const auto start = this->now();

    while (rclcpp::ok()) {
      std::array<AxisStatus, 3> status{};
      if (read_axis_status(status)) {
        const bool left_ready =
          status[static_cast<std::size_t>(left_wheel_axis_ - 1)].bControlLoopClosed;
        const bool right_ready =
          status[static_cast<std::size_t>(right_wheel_axis_ - 1)].bControlLoopClosed;
        const bool rail_ready =
          status[static_cast<std::size_t>(rail_axis_ - 1)].bControlLoopClosed;

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

      if ((this->now() - start).seconds() >= timeout_sec) {
        return false;
      }

      rclcpp::sleep_for(100ms);
    }

    return false;
  }

  bool configure_power_for_current_state_blocking(double timeout_sec)
  {
    // Vendor-safe order:
    //   StopMove -> bNotMoving + near-zero velocity -> PowerDisable -> PowerEnable.
    if (!wait_until_all_axes_stopped(timeout_sec, "power-mode handover")) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Power handover aborted because an axis did not stop.");
      return false;
    }

    power_disable_all_axes();

    if (rail_mode_requested()) {
      if (!power_enable_axis(rail_axis_)) {
        return false;
      }
      if (!wait_for_expected_power_state(false, true, timeout_sec)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Power state timeout: expected Wheels=DISABLE, Rail=ENABLE for rail_state=%s",
          rail_state_.c_str());
        return false;
      }
    } else {
      if (!power_enable_axis(left_wheel_axis_) ||
          !power_enable_axis(right_wheel_axis_))
      {
        return false;
      }
      if (!wait_for_expected_power_state(true, false, timeout_sec)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Power state timeout: expected Wheels=ENABLE, Rail=DISABLE for rail_state=%s",
          rail_state_.c_str());
        return false;
      }
    }

    return true;
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

    // Vendor-recommended mode:
    // Do NOT poll ReadAxisStatus() from the ROS timer/thread.  Register the
    // Beckhoff ADS notification once and use the asynchronously updated cache.
    AxisStatus_Notification(&axis_status_notification_cache_);
    axis_status_notification_registered_ = true;
    axis_status_notification_registered_at_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      this->get_logger(),
      "[ADS NOTIFY] AxisStatus notification registered: "
      "MotionHandler.fbAxisHandler.starAxisStatus");

    // The vendor sample waits while the notification updates axisStatusList.
    // Give the first SERVERCYCLE samples time to arrive before startup checks.
    std::this_thread::sleep_for(500ms);

    start_environment_sensor_notification();

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

    if (!power_enable_on_start_) {
      power_disable_all_axes();
      wheel_axes_enabled_ = false;
      rail_axis_enabled_ = false;
      motion_ready_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "power_enable_on_start=false. All axes remain disabled.");
      return;
    }

    // Startup state defaults to OUT_RAIL:
    //   Wheel Axis1/2 ENABLE, Rail Axis3 DISABLE.
    // This directly enforces the vendor requirement that the rail motor is
    // disabled while the AGV approaches and climbs onto the rail.
    if (!configure_power_for_current_state_blocking(5.0)) {
      motion_ready_ = false;
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
    // Allow the first STOP request of a new transition to be sent immediately.
    stop_attempt_initialized_.fill(false);

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
      status[static_cast<std::size_t>(left_wheel_axis_ - 1)].bControlLoopClosed;
    const bool right_ready =
      status[static_cast<std::size_t>(right_wheel_axis_ - 1)].bControlLoopClosed;
    const bool rail_ready =
      status[static_cast<std::size_t>(rail_axis_ - 1)].bControlLoopClosed;

    wheel_axes_enabled_ = left_ready && right_ready;
    rail_axis_enabled_ = rail_ready;

    switch (power_transition_stage_) {
      case PowerTransitionStage::NONE:
        return;

      case PowerTransitionStage::STOP_WHEELS:
      {
        const auto & left_status =
          status[static_cast<std::size_t>(left_wheel_axis_ - 1)];
        const auto & right_status =
          status[static_cast<std::size_t>(right_wheel_axis_ - 1)];

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

        if (left_stopped && right_stopped) {
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
          status[static_cast<std::size_t>(rail_axis_ - 1)];

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

        if (rail_stopped) {
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
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      RCLCPP_WARN(this->get_logger(), "Invalid /cmd_vel received.");
      return;
    }

    last_cmd_vel_time_ = this->now();

    if (safe_stop_latched_ || fault_latched_) {
      requested_linear_x_mps_ = 0.0;
      requested_angular_z_rps_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[SAFE STOP LATCH] /cmd_vel ignored until /agv/reset_faults succeeds.");
      return;
    }

    wheel_motion_armed_ =
      (std::fabs(msg->linear.x) > 1.0e-6 || std::fabs(msg->angular.z) > 1.0e-6);

    // Wheels must not receive a stored non-zero command while the robot is
    // ON_RAIL/WORKING/EXITING. A fresh /cmd_vel is required after OUT_RAIL.
    if (!wheel_mode_requested() ||
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
    const double speed_mps = static_cast<double>(msg->data);

    if (!std::isfinite(speed_mps)) {
      RCLCPP_WARN(this->get_logger(), "Invalid /rail/cmd_speed received.");
      return;
    }

    last_rail_cmd_time_ = this->now();

    if (safe_stop_latched_ || fault_latched_) {
      requested_rail_speed_mps_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[SAFE STOP LATCH] /rail/cmd_speed ignored until /agv/reset_faults succeeds.");
      return;
    }

    rail_motion_armed_ = (std::fabs(speed_mps) > 1.0e-6);

    // Rail Axis3 is physically disabled before ON_RAIL. Never retain an old
    // non-zero rail command that could execute after PowerEnable.
    if (!rail_mode_requested() ||
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

    const bool previous_rail_mode = rail_mode_requested();
    const std::string previous_state = rail_state_;
    rail_state_ = new_state;
    const bool new_rail_mode = rail_mode_requested();

    RCLCPP_INFO(
      this->get_logger(),
      "Rail state: %s -> %s",
      previous_state.c_str(),
      rail_state_.c_str());

    // Only a change between ground-drive mode and rail-drive mode requires
    // a servo power handover. WORKING <-> EXITING remains Rail mode.
    if (previous_rail_mode != new_rail_mode) {
      begin_power_transition_for_state();
    }
  }

  void wheel_power_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data && (fault_latched_ || safe_stop_latched_)) {
      response->success = false;
      response->message =
        "Wheel PowerEnable rejected: fault/safe-stop latch is active. "
        "Call /agv/reset_faults first.";
      return;
    }

    clear_motion_requests();
    wheel_motion_armed_ = false;
    rail_motion_armed_ = false;

    if (mock_mode_) {
      if (request->data) {
        wheel_axes_enabled_ = true;
        rail_axis_enabled_ = false;
        response->message = "Mock: wheel power ON, rail power OFF.";
      } else {
        wheel_axes_enabled_ = false;
        response->message = "Mock: wheel power OFF.";
      }
      response->success = true;
      return;
    }

    if (!ads_connected_) {
      response->success = false;
      response->message = "ADS is not connected.";
      return;
    }

    if (!wait_until_all_axes_stopped(stop_wait_timeout_sec_, "manual wheel power")) {
      response->success = false;
      response->message =
        "Wheel power request aborted: one or more axes did not physically stop.";
      return;
    }

    bool ok = true;

    if (request->data) {
      // Never allow wheel and rail servo power at the same time.
      if (rail_axis_enabled_) {
        ok = power_disable_axis(rail_axis_) && ok;
      }

      ok = power_enable_axis(left_wheel_axis_) && ok;
      ok = power_enable_axis(right_wheel_axis_) && ok;

      if (ok && wait_for_expected_power_state(true, false, 5.0)) {
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

    ok = power_disable_axis(left_wheel_axis_) && ok;
    ok = power_disable_axis(right_wheel_axis_) && ok;

    if (ok) {
      wheel_axes_enabled_ = false;
      response->success = true;
      response->message = "Wheel power OFF.";
    } else {
      response->success = false;
      response->message = "One or more wheel PowerDisable calls failed.";
    }
  }

  void rail_power_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data && (fault_latched_ || safe_stop_latched_)) {
      response->success = false;
      response->message =
        "Rail PowerEnable rejected: fault/safe-stop latch is active. "
        "Call /agv/reset_faults first.";
      return;
    }

    clear_motion_requests();
    wheel_motion_armed_ = false;
    rail_motion_armed_ = false;

    if (mock_mode_) {
      if (request->data) {
        wheel_axes_enabled_ = false;
        rail_axis_enabled_ = true;
        response->message = "Mock: rail power ON, wheel power OFF.";
      } else {
        rail_axis_enabled_ = false;
        response->message = "Mock: rail power OFF.";
      }
      response->success = true;
      return;
    }

    if (!ads_connected_) {
      response->success = false;
      response->message = "ADS is not connected.";
      return;
    }

    if (!wait_until_all_axes_stopped(stop_wait_timeout_sec_, "manual rail power")) {
      response->success = false;
      response->message =
        "Rail power request aborted: one or more axes did not physically stop.";
      return;
    }

    bool ok = true;

    if (request->data) {
      // Never allow wheel and rail servo power at the same time.
      if (wheel_axes_enabled_) {
        ok = power_disable_axis(left_wheel_axis_) && ok;
        ok = power_disable_axis(right_wheel_axis_) && ok;
      }

      ok = power_enable_axis(rail_axis_) && ok;

      if (ok && wait_for_expected_power_state(false, true, 5.0)) {
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

    ok = power_disable_axis(rail_axis_) && ok;

    if (ok) {
      rail_axis_enabled_ = false;
      response->success = true;
      response->message = "Rail power OFF.";
    } else {
      response->success = false;
      response->message = "Rail PowerDisable failed.";
    }
  }

  void reset_faults_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (mock_mode_) {
      motion_ready_ = true;
      fault_latched_ = false;
      safe_stop_latched_ = false;
      safe_stop_active_ = false;
      safe_stop_reason_.clear();
      wheel_motion_armed_ = false;
      rail_motion_armed_ = false;
      response->success = true;
      response->message = "Mock mode: faults cleared.";
      return;
    }

    if (!ads_connected_) {
      response->success = false;
      response->message = "ADS is not connected.";
      return;
    }

    // Safety first: no new motion command may survive the reset.
    clear_motion_requests();
    motion_ready_ = false;

    // Do NOT PowerDisable a moving/busy axis.
    // Axis2 may return ADSERR_DEVICE_BUSY (0x98110708) to StopMove.
    // Retry STOP with backoff and require vendor bNotMoving + near-zero speed.
    if (!wait_until_all_axes_stopped(stop_wait_timeout_sec_, "fault reset")) {
      response->success = false;
      response->message =
        "Fault reset aborted: one or more axes did not physically stop. "
        "PowerDisable/Reset was not forced on a moving axis.";
      return;
    }

    bool reset_ok = true;
    for (const int axis : {left_wheel_axis_, right_wheel_axis_, rail_axis_}) {
      const int32_t result = Reset(static_cast<uint32_t>(axis));
      if (result != NO_ERR_TcMCFUN) {
        reset_ok = false;
        RCLCPP_ERROR(
          this->get_logger(),
          "Reset(Axis%d) failed: 0x%X",
          axis,
          static_cast<unsigned int>(result));
      }
    }

    if (!reset_ok) {
      response->success = false;
      response->message = "One or more Reset() calls failed.";
      return;
    }

    if (!wait_until_faults_clear(3.0)) {
      response->success = false;
      response->message =
        "Reset command was sent, but an axis fault did not clear within 3 seconds.";
      return;
    }

    // Axes were verified stopped above, so the state-based PowerDisable /
    // PowerEnable handover is now safe.
    if (!configure_power_for_current_state_blocking(5.0)) {
      response->success = false;
      response->message =
        "Faults cleared, but safe state-based PowerEnable/PowerDisable failed.";
      return;
    }

    power_transition_stage_ = PowerTransitionStage::NONE;
    power_stage_command_sent_ = false;
    fault_latched_ = false;
    fault_stop_active_ = false;
    safe_stop_latched_ = false;
    safe_stop_active_ = false;
    safe_stop_reason_.clear();
    move_attempt_initialized_.fill(false);
    wheel_motion_armed_ = false;
    rail_motion_armed_ = false;
    motion_ready_ = true;

    last_cmd_vel_time_ = this->now();
    last_rail_cmd_time_ = this->now();

    response->success = true;
    response->message =
      "Reset completed after physical stop confirmation; power policy restored.";
  }

  // ============================================================
  // Beckhoff environmental sensor
  // ============================================================
  void start_environment_sensor_notification()
  {
    if (!enable_environment_sensor_ || mock_mode_ || !ads_connected_) {
      return;
    }

    try {
      // Vendor wrapper creates an ADS server-cycle notification for:
      //   Sensor_Modbus.SensorInfo
      // at 1 second cycle and updates the supplied SensorInfo object.
      Sensor_Notification(&environment_sensor_data_);

      environment_notification_registered_ = true;
      environment_notification_start_time_ = this->now();
      last_environment_publish_time_ = this->now();

      RCLCPP_INFO(
        this->get_logger(),
        "[ENV] ADS notification registered: Sensor_Modbus.SensorInfo");
    }
    catch (const std::exception & e) {
      environment_notification_registered_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "[ENV] Failed to register Sensor_Modbus.SensorInfo notification: %s. "
        "Motor control will continue normally.",
        e.what());
    }
    catch (...) {
      environment_notification_registered_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "[ENV] Failed to register Sensor_Modbus.SensorInfo notification. "
        "Motor control will continue normally.");
    }
  }

  void publish_environment_sensor_if_due(const rclcpp::Time & now)
  {
    if (
      !enable_environment_sensor_ ||
      mock_mode_ ||
      !ads_connected_ ||
      !environment_notification_registered_)
    {
      return;
    }

    // The vendor notification itself runs on a 1-second server cycle.
    // Give it one full cycle before publishing the first ROS value so that
    // startup-zero data is less likely to be mistaken for a real measurement.
    if ((now - environment_notification_start_time_).seconds() < 1.1) {
      return;
    }

    if (
      (now - last_environment_publish_time_).seconds() <
      environment_publish_period_sec_)
    {
      return;
    }
    last_environment_publish_time_ = now;

    const float solar =
      static_cast<float>(environment_sensor_data_.nSolar_Radiation);
    const float temperature =
      static_cast<float>(environment_sensor_data_.rTemperature);
    const float humidity =
      static_cast<float>(environment_sensor_data_.rHumidity);
    const float co2 =
      static_cast<float>(environment_sensor_data_.nCO2_PPM2);

    if (
      !std::isfinite(solar) ||
      !std::isfinite(temperature) ||
      !std::isfinite(humidity) ||
      !std::isfinite(co2))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "[ENV] Invalid environmental sensor value received; ROS publish skipped.");
      return;
    }

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
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "[ENV] solar=%.1f W/m^2 temp=%.2f C humidity=%.2f %% CO2=%.0f ppm",
        static_cast<double>(solar),
        static_cast<double>(temperature),
        static_cast<double>(humidity),
        static_cast<double>(co2));
    }
  }

  // ============================================================
  // Main timer
  // ============================================================
  void timer_callback()
  {
    const auto now = this->now();

    if (mock_mode_) {
      publish_mock_feedback();
    } else if (ads_connected_) {
      std::array<AxisStatus, 3> status{};
      if (read_axis_status(status)) {
        process_axis_status(status);
        if (safe_stop_latched_) {
          process_safe_stop(status);
        }
        if (!fault_latched_ && !safe_stop_latched_) {
          process_power_transition(status);
        }
      }
    }

    double target_linear = requested_linear_x_mps_;
    double target_angular = requested_angular_z_rps_;
    double target_rail = requested_rail_speed_mps_;

    const bool wheel_timeout =
      (now - last_cmd_vel_time_).seconds() > cmd_vel_timeout_sec_;
    const bool rail_timeout =
      (now - last_rail_cmd_time_).seconds() > rail_cmd_timeout_sec_;

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
        RCLCPP_WARN(
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
        RCLCPP_WARN(
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
      } else if (rail_mode_requested()) {
        apply_mock_commands(0.0, 0.0, target_rail);
      } else {
        apply_mock_commands(target_linear, target_angular, 0.0);
      }
    } else if (ads_connected_ && motion_ready_ && !fault_latched_ && !safe_stop_latched_) {
      if (!power_transition_active && wheel_mode_requested() && wheel_axes_enabled_) {
        apply_wheel_command(target_linear, target_angular);
      } else {
        applied_left_mps_ = 0.0;
        applied_right_mps_ = 0.0;
      }

      if (!power_transition_active && rail_mode_requested() && rail_axis_enabled_) {
        apply_rail_command(target_rail);
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
    // V6: Notification only.  No ReadAxisStatus() ADS polling is performed.
    // TcMCApi::AxisStatus_NotifyCallback() asynchronously memcpy()s the
    // PLC starAxisStatus array into axis_status_notification_cache_.
    if (!ads_connected_ || !axis_status_notification_registered_) {
      return false;
    }

    status = axis_status_notification_cache_;
    return true;
  }

  bool has_any_fault(const std::array<AxisStatus, 3> & status) const
  {
    return
      status[static_cast<std::size_t>(left_wheel_axis_ - 1)].bError ||
      status[static_cast<std::size_t>(right_wheel_axis_ - 1)].bError ||
      status[static_cast<std::size_t>(rail_axis_ - 1)].bError;
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
      status[static_cast<std::size_t>(left_wheel_axis_ - 1)];
    const auto & right =
      status[static_cast<std::size_t>(right_wheel_axis_ - 1)];
    const auto & rail =
      status[static_cast<std::size_t>(rail_axis_ - 1)];

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
      "READY L=%d R=%d | NM L=%d R=%d | "
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

    if (std::fabs(left_cmd_mps) >= kCommandCheckMinMps) {
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

    if (std::fabs(right_cmd_mps) >= kCommandCheckMinMps) {
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

  void process_axis_status(const std::array<AxisStatus, 3> & status)
  {
    log_axis_monitor(status);

    if (has_any_fault(status)) {
      const auto & left =
        status[static_cast<std::size_t>(left_wheel_axis_ - 1)];
      const auto & right =
        status[static_cast<std::size_t>(right_wheel_axis_ - 1)];
      const auto & rail =
        status[static_cast<std::size_t>(rail_axis_ - 1)];

      if (!fault_latched_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Axis fault detected. Left(error=%d code=0x%X), "
          "Right(error=%d code=0x%X), Rail(error=%d code=0x%X). "
          "New motion is inhibited. STOP will be retried safely; "
          "PowerDisable is forbidden until physical stop is confirmed.",
          left.bError ? 1 : 0,
          static_cast<unsigned int>(left.nErrorCode),
          right.bError ? 1 : 0,
          static_cast<unsigned int>(right.nErrorCode),
          rail.bError ? 1 : 0,
          static_cast<unsigned int>(rail.nErrorCode));

        clear_motion_requests();
        fault_stop_active_ = true;
        enter_safe_stop("axis fault detected");
      }

      fault_latched_ = true;
      motion_ready_ = false;

      // Continue STOP attempts while faulted. Device BUSY is retried with
      // backoff; Controller Enable stays ON until bNotMoving + v~=0.
      if (fault_stop_active_) {
        const bool left_stopped =
          axis_is_physically_stopped(left, left_wheel_sign_);
        const bool right_stopped =
          axis_is_physically_stopped(right, right_wheel_sign_);
        const bool rail_stopped =
          axis_is_physically_stopped(rail, rail_sign_);

        if (!left_stopped) {
          (void)request_axis_stop(
            left_wheel_axis_, wheel_ramp_ms_, "axis fault");
        }
        if (!right_stopped) {
          (void)request_axis_stop(
            right_wheel_axis_, wheel_ramp_ms_, "axis fault");
        }
        if (!rail_stopped) {
          (void)request_axis_stop(
            rail_axis_, rail_ramp_ms_, "axis fault");
        }

        if (left_stopped && right_stopped && rail_stopped) {
          fault_stop_active_ = false;
          applied_left_mps_ = 0.0;
          applied_right_mps_ = 0.0;
          applied_rail_mps_ = 0.0;
          RCLCPP_WARN(
            this->get_logger(),
            "[SAFE STOP] Faulted system is now physically stopped. "
            "Call /agv/reset_faults after removing the hardware cause.");
        }
      }

      return;
    }

    const auto & left_status =
      status[static_cast<std::size_t>(left_wheel_axis_ - 1)];
    const auto & right_status =
      status[static_cast<std::size_t>(right_wheel_axis_ - 1)];
    const auto & rail_status =
      status[static_cast<std::size_t>(rail_axis_ - 1)];

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

  // ============================================================
  // Wheel control
  // ============================================================
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
      const bool left_stopped =
        std::fabs(actual_left_mps_) <= stopped_velocity_threshold_mps_;
      const bool right_stopped =
        std::fabs(actual_right_mps_) <= stopped_velocity_threshold_mps_;

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

    const double left_target_mps =
      linear_mps - angular_rps * wheel_separation_m_ / 2.0;
    const double right_target_mps =
      linear_mps + angular_rps * wheel_separation_m_ / 2.0;

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
        status[static_cast<std::size_t>(rail_axis_ - 1)];
      rail_b_not_moving = rail_status.bNotMoving;
    }

    const bool rail_physically_stopped =
      rail_b_not_moving &&
      std::fabs(actual_rail_mps_) <= kStoppedThresholdMps;

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

        // If the previous StopMove attempt returned BUSY, wait at least one
        // rail ramp interval before trying again.  This avoids repeatedly
        // colliding with an active MC command.
        if (rail_stop_retry_wait_active_) {
          const double retry_wait_ms =
            std::chrono::duration<double, std::milli>(
              now_tp - rail_stop_retry_wait_start_tp_).count();

          if (retry_wait_ms < rail_ramp_ms_) {
            return;
          }

          rail_stop_retry_wait_active_ = false;
        }

        if (!move_ramp_complete) {
          return;
        }

        RCLCPP_INFO(
          this->get_logger(),
          "[RAIL STOP TX] Axis%d ramp=%.0f ms pending_target=%+.3f m/s",
          rail_axis_,
          rail_ramp_ms_,
          rail_pending_target_mps_);

        // IMPORTANT: g/STOP only calls StopMove for Axis3.
        const int32_t result =
          StopMove(
            static_cast<uint32_t>(rail_axis_),
            rail_ramp_ms_,
            jerk_percent_);

        if (result == NO_ERR_TcMCFUN) {
          rail_stop_tx_sent_ = true;
          rail_stop_tx_tp_ = now_tp;

          RCLCPP_INFO(
            this->get_logger(),
            "[RAIL STOP] Axis%d StopMove accepted; waiting for "
            "AxisStatus Notification bNotMoving",
            rail_axis_);

          return;
        }

        if (is_ads_device_busy(result)) {
          rail_stop_retry_wait_active_ = true;
          rail_stop_retry_wait_start_tp_ = now_tp;

          RCLCPP_WARN(
            this->get_logger(),
            "[RAIL STOP] Axis%d StopMove BUSY (0x98110708). "
            "Will not spam StopMove; retry after rail ramp %.0f ms.",
            rail_axis_,
            rail_ramp_ms_);
          return;
        }

        RCLCPP_ERROR(
          this->get_logger(),
          "[RAIL STOP] Axis%d StopMove failed: 0x%X",
          rail_axis_,
          static_cast<unsigned int>(result));
        return;
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

      if (
        std::fabs(applied_rail_mps_) <= kZeroMps &&
        rail_physically_stopped)
      {
        applied_rail_mps_ = 0.0;
        return;
      }

      rail_stop_sequence_active_ = true;
      rail_waiting_for_stop_ = true;
      rail_stop_tx_sent_ = false;
      rail_stop_retry_wait_active_ = false;
      return;
    }

    // ---------------------------------------------------------------------
    // Direction reversal: Axis3 STOP first.
    // ---------------------------------------------------------------------
    if (
      std::fabs(applied_rail_mps_) > kZeroMps &&
      rail_target_mps * applied_rail_mps_ < 0.0)
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

    // New non-zero Axis3 MoveVelocity.
    const double plc_velocity_mmps =
      rail_target_mps * velocity_scale_ * rail_sign_;

    RCLCPP_INFO(
      this->get_logger(),
      "[MOVE TX] Axis%d target=%+.1f mm/s",
      rail_axis_,
      plc_velocity_mmps);

    const int32_t result =
      MoveVelocity(
        static_cast<uint32_t>(rail_axis_),
        plc_velocity_mmps,
        rail_ramp_ms_,
        jerk_percent_);

    if (result == NO_ERR_TcMCFUN) {
      applied_rail_mps_ = rail_target_mps;
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

    // ------------------------------------------------------------
    // A) Direction-change STOP already accepted.
    //    Wait for physical stop before issuing opposite direction.
    // ------------------------------------------------------------
    if (waiting_for_stop) {
      pending_target_mps = target_mps;

      if (std::fabs(actual_mps) > stopped_velocity_threshold_mps_) {
        return;
      }

      // Physical velocity is now near zero.
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

      if (
        std::fabs(applied_mps) <= kZeroMps &&
        std::fabs(actual_mps) <= stopped_velocity_threshold_mps_)
      {
        waiting_for_stop = false;
        applied_mps = 0.0;
        return;
      }

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
      std::fabs(applied_mps) > kZeroMps &&
      target_mps * applied_mps < 0.0)
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

    // Re-check the safety latch immediately before the vendor API call.
    if (fault_latched_ || safe_stop_latched_ || !motion_ready_) {
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

    const int32_t result =
      MoveVelocity(
        static_cast<uint32_t>(axis),
        plc_velocity_mmps,
        ramp_ms,
        jerk_percent_);

    if (result == NO_ERR_TcMCFUN) {
      applied_mps = target_mps;

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
            status[static_cast<std::size_t>(axis - 1)];
          stopped = axis_is_physically_stopped(s, sign);
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

    // Axis feedback is not trustworthy for localization after a drive fault.
    // Keep the last odom pose and publish zero twist until reset succeeds.
    const bool freeze_odom = fault_latched_ || safe_stop_latched_;
    const double linear_mps = freeze_odom ? 0.0 : (left_mps + right_mps) / 2.0;
    const double angular_rps = freeze_odom ? 0.0 :
      (right_mps - left_mps) / wheel_separation_m_;

    if (!freeze_odom && dt > 0.0) {
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
    } else if (!ads_connected_) {
      msg.data = "AGV ADS Driver: ADS DISCONNECTED";
    } else if (safe_stop_latched_) {
      msg.data = "AGV ADS Driver: SAFE STOP LATCHED / " + safe_stop_reason_;
    } else if (fault_latched_) {
      msg.data = "AGV ADS Driver: AXIS FAULT LATCHED";
    } else if (!motion_ready_) {
      msg.data = "AGV ADS Driver: CONNECTED / MOTION NOT READY";
    } else {
      msg.data =
        "AGV ADS Driver: READY / rail_state=" + rail_state_ +
        " / wheels=" + std::string(wheel_axes_enabled_ ? "ENABLE" : "DISABLE") +
        " / rail=" + std::string(rail_axis_enabled_ ? "ENABLE" : "DISABLE");
    }

    status_pub_->publish(msg);
  }

private:
  bool mock_mode_{true};
  bool ads_connected_{false};
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

  // Mechanical mode / servo power safety interlock.
  // Ground mode: Wheels ENABLE, Rail DISABLE.
  // Rail mode (ON_RAIL/WORKING/EXITING): Wheels DISABLE, Rail ENABLE.
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

  double wheel_separation_m_{1.0};
  double wheel_diameter_m_{0.206};
  double velocity_scale_{1000.0};

  double wheel_ramp_ms_{1000.0};
  double rail_ramp_ms_{1000.0};
  double rail_direction_stop_hold_ms_{200.0};
  double jerk_percent_{100.0};

  double cmd_vel_timeout_sec_{10.0};
  double rail_cmd_timeout_sec_{10.0};

  double wheel_command_epsilon_mps_{0.005};
  double wheel_move_command_min_interval_ms_{100.0};
  double wheel_stop_pending_timeout_sec_{4.0};
  double rail_command_epsilon_mps_{0.001};

  bool power_enable_on_start_{true};
  bool power_disable_on_shutdown_{true};
  bool log_motion_commands_{false};

  bool log_axis_monitor_{true};
  double axis_monitor_period_sec_{0.5};

  // Environmental sensor over Beckhoff ADS.
  bool enable_environment_sensor_{true};
  double environment_publish_period_sec_{1.0};
  bool log_environment_sensor_{true};
  bool environment_notification_registered_{false};
  SensorInfo environment_sensor_data_{};

  double stop_retry_interval_ms_{1000.0};
  double stop_wait_timeout_sec_{4.0};
  double stopped_velocity_threshold_mps_{0.01};

  // Index 1..3 maps directly to Beckhoff Axis1..Axis3.
  std::array<std::chrono::steady_clock::time_point, 4> last_stop_attempt_tp_{};
  std::array<bool, 4> stop_attempt_initialized_{{false, false, false, false}};
  std::array<std::chrono::steady_clock::time_point, 4> last_move_attempt_tp_{};
  std::array<bool, 4> move_attempt_initialized_{{false, false, false, false}};

  bool wheel_stop_pending_{false};
  std::chrono::steady_clock::time_point wheel_stop_pending_since_{};

  // Beckhoff ADS Notification cache. Registered once after Connect().
  // This replaces periodic ReadAxisStatus() polling.
  std::array<AxisStatus, 3> axis_status_notification_cache_{};
  bool axis_status_notification_registered_{false};
  std::chrono::steady_clock::time_point axis_status_notification_registered_at_{};

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
  rclcpp::Time last_feedback_time_;
  rclcpp::Time last_axis_monitor_time_;
  rclcpp::Time last_environment_publish_time_;
  rclcpp::Time environment_notification_start_time_;
  rclcpp::Time rail_zero_hold_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_theta_{0.0};

  double left_joint_position_rad_{0.0};
  double right_joint_position_rad_{0.0};

  std::size_t status_counter_{0};

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

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_faults_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr wheel_power_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr rail_power_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
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