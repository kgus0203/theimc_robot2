#include "theimc_bt_nodes/save_rail_progress.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

#include "theimc_bt_nodes/rail_progress_store.hpp"

namespace theimc_bt_nodes
{

SaveRailProgress::SaveRailProgress(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(xml_tag_name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error(
            "SaveRailProgress: ROS node not in blackboard");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/rail/odom",
    rclcpp::QoS(10),
    std::bind(
      &SaveRailProgress::odomCallback,
      this,
      std::placeholders::_1),
    options);
}

BT::PortsList SaveRailProgress::providedPorts()
{
  return {
    BT::InputPort<int>("rail_id"),
    BT::InputPort<std::string>(
      "progress_file",
      "",
      "Empty = ~/.ros/theimc_rail_progress.yaml"),
    // 3개 인자로 수정 (이름, 기본값, 설명)
    BT::InputPort<bool>("completed", false, "Whether the mission is completed"),
    BT::InputPort<bool>(
      "force",
      false,
      "true saves exact current distance"),
    BT::InputPort<bool>(
      "mark_resume",
      false,
      "true marks this rail as interrupted/resume target"),
    // 3개 인자로 수정 (이름, 기본값, 설명)
    BT::InputPort<double>("odom_stale_sec", 0.5, "Odom stale timeout in seconds"),
    BT::OutputPort<double>("saved_progress_m")
  };
}

void SaveRailProgress::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double distance =
    static_cast<double>(msg->pose.pose.position.x);

  if (!std::isfinite(distance)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  current_distance_m_ = distance;
  odom_time_ = node_->now();
  odom_received_ = true;
}

bool SaveRailProgress::getFreshDistance(
  double max_age_sec,
  double & distance_m)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!odom_received_) {
    return false;
  }

  if ((node_->now() - odom_time_).seconds() > max_age_sec) {
    return false;
  }

  distance_m = current_distance_m_;
  return true;
}

BT::NodeStatus SaveRailProgress::tick()
{
  callback_group_executor_.spin_some();

  int rail_id = 0;
  std::string requested_file;
  bool completed = false;
  bool force = false;
  bool mark_resume = false;
  double stale_sec = 0.5;

  if (!getInput("rail_id", rail_id) || rail_id <= 0) {
    throw BT::RuntimeError(
            "SaveRailProgress: missing [rail_id]");
  }

  getInput("progress_file", requested_file);
  getInput("completed", completed);
  getInput("force", force);
  getInput("mark_resume", mark_resume);
  getInput("odom_stale_sec", stale_sec);

  double current = 0.0;

  if (!getFreshDistance(stale_sec, current)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[SaveRailProgress] No fresh /rail/odom");
    return BT::NodeStatus::FAILURE;
  }

  current = std::max(0.0, current);

  const auto path =
    resolveRailProgressFile(requested_file);

  const auto old_record =
    readRailProgress(path, rail_id);

  RailProgressRecord new_record;
  new_record.completed = completed;

  // Manual interruption must use force=true so EXACT stop position is saved.
  new_record.progress_m =
    force ? current : std::max(old_record.progress_m, current);

  writeRailProgress(
    path,
    rail_id,
    new_record,
    mark_resume);

  setOutput(
    "saved_progress_m",
    new_record.progress_m);

  RCLCPP_INFO(
    node_->get_logger(),
    "[SaveRailProgress] rail=%d progress=%.3f m resume=%s",
    rail_id,
    new_record.progress_m,
    mark_resume ? "true" : "false");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace theimc_bt_nodes
