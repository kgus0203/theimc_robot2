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

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;

  tof_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/tof_distance",
    rclcpp::QoS(10),
    std::bind(
      &RailEnter::tofCallback,
      this,
      std::placeholders::_1),
    subscription_options);

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(10));

  cmd_rail_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_rail",
    rclcpp::QoS(10));

  rail_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/rail_state",
    rclcpp::QoS(10));
}

BT::PortsList RailEnter::providedPorts()
{
  return {
    // TOF <= enter_tof_mm:
    // 레일 진입부를 지나고 있다고 판단
    BT::InputPort<double>(
      "enter_tof_mm",
      120.0,
      "TOF <= threshold means rail entry has started"),

    // 반드시 위의 낮은 TOF를 먼저 본 뒤에만 이 조건을 사용한다.
    // TOF >= on_rail_tof_mm:
    // 레일 위에 완전히 올라왔다고 판단
    BT::InputPort<double>(
      "on_rail_tof_mm",
      160.0,
      "After ENTERING was detected, TOF >= threshold means ON_RAIL"),

    BT::InputPort<double>(
      "settle_sec",
      2.0,
      "Stop/wait time after ENTERING detection"),

    BT::InputPort<double>(
      "rail_velocity",
      0.3,
      "Rail motor forward velocity while entering"),

    BT::InputPort<double>(
      "timeout_sec",
      0.0,
      "Whole RailEnter timeout. 0 means no timeout"),

    BT::InputPort<double>(
      "tof_stale_sec",
      0.5,
      "Maximum accepted TOF sample age"),

    BT::InputPort<int>(
      "enter_confirm_samples",
      2,
      "Consecutive low TOF samples required for ENTERING"),

    BT::InputPort<int>(
      "on_rail_confirm_samples",
      3,
      "Consecutive high TOF samples required for ON_RAIL")
  };
}

void RailEnter::tofCallback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  const double distance_mm = static_cast<double>(msg->data);

  // STM ToF 오류값(-1 등) 또는 NaN/Inf 무시
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
  getInput("settle_sec", settle_sec_);
  getInput("rail_velocity", rail_velocity_);
  getInput("timeout_sec", timeout_sec_);
  getInput("tof_stale_sec", tof_stale_sec_);
  getInput("enter_confirm_samples", enter_confirm_samples_);
  getInput("on_rail_confirm_samples", on_rail_confirm_samples_);

  if (
    enter_tof_mm_ <= 0.0 ||
    on_rail_tof_mm_ <= enter_tof_mm_ ||
    settle_sec_ < 0.0 ||
    rail_velocity_ <= 0.0 ||
    timeout_sec_ < 0.0 ||
    tof_stale_sec_ <= 0.0 ||
    enter_confirm_samples_ < 1 ||
    on_rail_confirm_samples_ < 1)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[RailEnter] Invalid input parameters");

    stopOutputs();
    return BT::NodeStatus::FAILURE;
  }

  phase_ = Phase::WAIT_ENTRY_LOW;

  enter_low_count_ = 0;
  on_rail_high_count_ = 0;

  start_time_ = node_->now();
  phase_start_time_ = start_time_;

  // RailApproach 완료 직후이므로 진입 판단 전에는 안전하게 정지.
  publishBaseStop();
  publishRailVelocity(0.0);

  RCLCPP_INFO(
    node_->get_logger(),
    "[RailEnter] START - wait TOF <= %.1f mm (%d samples), "
    "then TOF >= %.1f mm (%d samples) => ON_RAIL",
    enter_tof_mm_,
    enter_confirm_samples_,
    on_rail_tof_mm_,
    on_rail_confirm_samples_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RailEnter::onRunning()
{
  // 이 노드 전용 callback group의 TOF callback 처리
  callback_group_executor_.spin_some();

  if (timedOut()) {
    stopOutputs();

    RCLCPP_ERROR(
      node_->get_logger(),
      "[RailEnter] Timeout");

    return BT::NodeStatus::FAILURE;
  }

  double tof_mm = 0.0;

  if (!getFreshTof(tof_mm)) {
    return BT::NodeStatus::RUNNING;
  }

  const auto now = node_->now();

  switch (phase_) {
    // ------------------------------------------------------------
    // RailApproach가 끝난 상태.
    //
    // 정상적으로는 여기서 TOF가 160~180 mm 정도이다.
    // 이 높은 값은 절대로 ON_RAIL로 인정하지 않는다.
    //
    // 먼저 100~120 mm 영역을 반드시 통과해야 한다.
    // ------------------------------------------------------------
    case Phase::WAIT_ENTRY_LOW:
    {
      publishBaseStop();
      publishRailVelocity(0.0);

      if (tof_mm <= enter_tof_mm_) {
        ++enter_low_count_;

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailEnter] entry-low candidate: %.1f mm (%d/%d)",
          tof_mm,
          enter_low_count_,
          enter_confirm_samples_);
      } else {
        enter_low_count_ = 0;
      }

      if (enter_low_count_ >= enter_confirm_samples_) {
        publishRailState("ENTERING");

        phase_ = Phase::SETTLING;
        phase_start_time_ = now;
        on_rail_high_count_ = 0;

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailEnter] state => ENTERING (TOF %.1f mm)",
          tof_mm);
      }

      return BT::NodeStatus::RUNNING;
    }

    // ------------------------------------------------------------
    // 진입부가 확인된 직후 잠시 정지.
    // 이전 STM sensor_process 동작의 settle 시간을 그대로 보존.
    // ------------------------------------------------------------
    case Phase::SETTLING:
    {
      publishBaseStop();
      publishRailVelocity(0.0);

      if ((now - phase_start_time_).seconds() >= settle_sec_) {
        phase_ = Phase::ENTERING;
        on_rail_high_count_ = 0;

        publishRailVelocity(rail_velocity_);

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailEnter] rail motor forward %.3f m/s; "
          "waiting for ON_RAIL TOF",
          rail_velocity_);
      }

      return BT::NodeStatus::RUNNING;
    }

    // ------------------------------------------------------------
    // 레일 모터가 실제로 로봇을 레일 위로 올리는 중.
    //
    // 여기까지 왔다는 것 자체가 이미 <=120 mm를 통과했다는 뜻.
    // 따라서 이제 다시 높은 값(기본 >=160 mm)이 나오면 ON_RAIL.
    //
    // 시간으로 ON_RAIL을 결정하지 않는다.
    // ------------------------------------------------------------
    case Phase::ENTERING:
    {
      publishBaseStop();
      publishRailVelocity(rail_velocity_);

      if (tof_mm >= on_rail_tof_mm_) {
        ++on_rail_high_count_;

        RCLCPP_INFO(
          node_->get_logger(),
          "[RailEnter] on-rail candidate: %.1f mm (%d/%d)",
          tof_mm,
          on_rail_high_count_,
          on_rail_confirm_samples_);
      } else {
        on_rail_high_count_ = 0;
      }

      if (on_rail_high_count_ >= on_rail_confirm_samples_) {
        publishRailVelocity(0.0);
        publishBaseStop();
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

  stopOutputs();
  return BT::NodeStatus::FAILURE;
}

void RailEnter::onHalted()
{
  stopOutputs();

  RCLCPP_WARN(
    node_->get_logger(),
    "[RailEnter] Halted - rail/base stop sent");
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

void RailEnter::publishBaseStop()
{
  geometry_msgs::msg::Twist msg;
  cmd_vel_pub_->publish(msg);
}

void RailEnter::publishRailVelocity(double velocity)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = velocity;

  cmd_rail_pub_->publish(msg);
}

void RailEnter::publishRailState(const std::string & state)
{
  std_msgs::msg::String msg;
  msg.data = state;

  rail_state_pub_->publish(msg);
}

void RailEnter::stopOutputs()
{
  publishRailVelocity(0.0);
  publishBaseStop();
}

}  // namespace theimc_bt_nodes