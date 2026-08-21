#include "theimc_bt_nodes/get_location_pose.hpp"

#include <exception>
#include <string>

#include "yaml-cpp/yaml.h"

namespace theimc_bt_nodes
{

GetLocationPose::GetLocationPose(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(xml_tag_name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  try {
    yaml_path_ = config.blackboard->get<std::string>("rails_yaml");
    loadLocations(yaml_path_);
  } catch (const std::exception & exception) {
    load_error_ = exception.what();
    RCLCPP_ERROR(
      node_->get_logger(), "GetLocationPose failed to load locations YAML: %s",
      load_error_.c_str());
  }
}

BT::PortsList GetLocationPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("location", "Location name"),
    BT::OutputPort<double>("x", "Location x coordinate"),
    BT::OutputPort<double>("y", "Location y coordinate"),
    BT::OutputPort<double>("yaw", "Location yaw in radians"),
  };
}

void GetLocationPose::loadLocations(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node locations = root["locations"];
  if (!locations || !locations.IsMap()) {
    throw std::runtime_error("missing top-level 'locations' map in " + yaml_path);
  }

  for (const auto & entry : locations) {
    const std::string name = entry.first.as<std::string>();
    const YAML::Node pose = entry.second;
    if (!pose["x"] || !pose["y"] || !pose["yaw"]) {
      throw std::runtime_error(
              "location '" + name + "' requires x, y, and yaw in " + yaml_path);
    }
    locations_[name] = Pose{
      pose["x"].as<double>(), pose["y"].as<double>(), pose["yaw"].as<double>()};
  }

  RCLCPP_INFO(
    node_->get_logger(), "GetLocationPose loaded %zu locations from %s",
    locations_.size(), yaml_path.c_str());
}

BT::NodeStatus GetLocationPose::tick()
{
  if (!load_error_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "GetLocationPose cannot run because YAML loading failed: %s",
      load_error_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string name;
  if (!getInput("location", name)) {
    RCLCPP_ERROR(node_->get_logger(), "GetLocationPose requires location input");
    return BT::NodeStatus::FAILURE;
  }

  const auto pose = locations_.find(name);
  if (pose == locations_.end()) {
    RCLCPP_ERROR(
      node_->get_logger(), "GetLocationPose location '%s' does not exist in %s",
      name.c_str(), yaml_path_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!setOutput("x", pose->second.x) || !setOutput("y", pose->second.y) ||
    !setOutput("yaw", pose->second.yaw))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "GetLocationPose failed to write pose outputs for '%s'", name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(), "GetLocationPose %s: x=%.3f, y=%.3f, yaw=%.3f",
    name.c_str(), pose->second.x, pose->second.y, pose->second.yaw);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace theimc_bt_nodes
