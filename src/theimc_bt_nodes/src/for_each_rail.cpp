#include "theimc_bt_nodes/for_each_rail.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <unordered_set>

namespace theimc_bt_nodes
{
namespace
{

std::string trim(const std::string & value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(), [](unsigned char character) {return std::isspace(character);});
  if (first == value.end()) {
    return {};
  }

  const auto last = std::find_if_not(
    value.rbegin(), value.rend(), [](unsigned char character) {return std::isspace(character);})
    .base();
  return std::string(first, last);
}

std::string joinRailIds(const std::vector<int> & rail_ids)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < rail_ids.size(); ++index) {
    if (index > 0) {
      stream << ',';
    }
    stream << rail_ids[index];
  }
  return stream.str();
}

}  // namespace

ForEachRail::ForEachRail(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(xml_tag_name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList ForEachRail::providedPorts()
{
  return {
    BT::InputPort<std::string>("selected_rails", "Comma-separated rail IDs"),
    BT::InputPort<std::string>("rail_list", "Alias for selected_rails"),
    BT::InputPort<int>("start_index", 0, "Index to resume from after interruption"),
    BT::OutputPort<int>("current_rail", "Rail ID currently being executed"),
    BT::OutputPort<int>("last_executed_index", "To save progress"),
  };
}

bool ForEachRail::initializeRailList()
{
  std::string selected_rails;
  const auto selected_result = getInput("selected_rails", selected_rails);
  if (!selected_result) {
    const auto list_result = getInput("rail_list", selected_rails);
    if (!list_result) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "ForEachRail requires either selected_rails or rail_list input");
      return false;
    }
  }

  selected_rails = trim(selected_rails);
  if (selected_rails.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "ForEachRail received an empty rail list");
    return false;
  }
  if (selected_rails.back() == ',') {
    RCLCPP_ERROR(
      node_->get_logger(), "ForEachRail invalid list '%s': trailing comma",
      selected_rails.c_str());
    return false;
  }

  std::vector<int> parsed_ids;
  std::unordered_set<int> seen_ids;
  std::stringstream stream(selected_rails);
  std::string token;

  while (std::getline(stream, token, ',')) {
    token = trim(token);
    if (token.empty()) {
      RCLCPP_ERROR(
        node_->get_logger(), "ForEachRail invalid list '%s': empty item",
        selected_rails.c_str());
      return false;
    }
    if (!std::all_of(
        token.begin(), token.end(),
        [](unsigned char character) {return std::isdigit(character);}))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "ForEachRail invalid rail ID '%s' in list '%s'",
        token.c_str(), selected_rails.c_str());
      return false;
    }

    std::size_t parsed_characters = 0;
    int rail_id = 0;
    try {
      rail_id = std::stoi(token, &parsed_characters);
    } catch (const std::exception &) {
      RCLCPP_ERROR(
        node_->get_logger(), "ForEachRail invalid rail ID '%s' in list '%s'",
        token.c_str(), selected_rails.c_str());
      return false;
    }

    if (parsed_characters != token.size()) {
      RCLCPP_ERROR(
        node_->get_logger(), "ForEachRail invalid rail ID '%s' in list '%s'",
        token.c_str(), selected_rails.c_str());
      return false;
    }
    if (rail_id < 1 || rail_id > 12) {
      RCLCPP_ERROR(
        node_->get_logger(), "ForEachRail rail ID %d is outside the valid range 1..12",
        rail_id);
      return false;
    }

    if (seen_ids.insert(rail_id).second) {
      parsed_ids.push_back(rail_id);
    } else {
      RCLCPP_WARN(node_->get_logger(), "ForEachRail ignoring duplicate rail ID %d", rail_id);
    }
  }

  if (parsed_ids.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "ForEachRail did not receive any valid rail IDs");
    return false;
  }

  rail_ids_ = std::move(parsed_ids);

  int start_idx = 0;
  if (!getInput("start_index", start_idx)) {
    start_idx = 0;
  }

  // start_index == rail_ids_.size() means all selected rails are already complete.
  // This must NOT reset to 0.
  if (start_idx < 0 || static_cast<std::size_t>(start_idx) > rail_ids_.size()) {
    RCLCPP_WARN(
      node_->get_logger(), "ForEachRail start_index %d is out of bounds. Defaulting to 0.",
      start_idx);
    start_idx = 0;
  }

  current_index_ = static_cast<std::size_t>(start_idx);
  initialized_ = true;

  RCLCPP_INFO(
    node_->get_logger(), "ForEachRail selected rails: [%s] (%zu total). Starting from index %d",
    joinRailIds(rail_ids_).c_str(), rail_ids_.size(), start_idx);
  return true;
}

BT::NodeStatus ForEachRail::tick()
{
  if (child_node_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "ForEachRail requires exactly one child node");
    return BT::NodeStatus::FAILURE;
  }

  if (!initialized_ && !initializeRailList()) {
    return BT::NodeStatus::FAILURE;
  }

  if (current_index_ >= rail_ids_.size()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ForEachRail no remaining rails: start/current index=%zu, total=%zu -> SUCCESS",
      current_index_,
      rail_ids_.size());
    return BT::NodeStatus::SUCCESS;
  }

  const int rail_id = rail_ids_[current_index_];
  if (!setOutput("current_rail", rail_id)) {
    RCLCPP_ERROR(node_->get_logger(), "ForEachRail failed to write current_rail output");
    return BT::NodeStatus::FAILURE;
  }
  if (!setOutput("last_executed_index", static_cast<int>(current_index_))) {
    RCLCPP_ERROR(node_->get_logger(), "ForEachRail failed to write last_executed_index output");
    return BT::NodeStatus::FAILURE;
  }

  if (child_node_->status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(
      node_->get_logger(), "ForEachRail executing rail %d (%zu/%zu)",
      rail_id, current_index_ + 1, rail_ids_.size());
  }

  const auto child_status = child_node_->executeTick();
  if (child_status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  if (child_status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(
      node_->get_logger(), "ForEachRail rail %d failed (%zu/%zu); aborting mission",
      rail_id, current_index_ + 1, rail_ids_.size());
    resetChild();
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(), "ForEachRail rail %d succeeded (%zu/%zu)",
    rail_id, current_index_ + 1, rail_ids_.size());
  resetChild();
  ++current_index_;

  setOutput("last_executed_index", static_cast<int>(current_index_));

  if (current_index_ == rail_ids_.size()) {
    RCLCPP_INFO(
      node_->get_logger(), "ForEachRail completed all %zu selected rails", rail_ids_.size());
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void ForEachRail::halt()
{
  if (initialized_) {
    RCLCPP_WARN(
      node_->get_logger(), "ForEachRail halted at item %zu/%zu; resetting traversal state",
      std::min(current_index_ + 1, rail_ids_.size()), rail_ids_.size());
  }
  BT::DecoratorNode::halt();
  clearState();
}

void ForEachRail::clearState()
{
  rail_ids_.clear();
  current_index_ = 0;
  initialized_ = false;
}

}  // namespace theimc_bt_nodes