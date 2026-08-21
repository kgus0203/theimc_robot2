#ifndef THEIMC_BT_NODES__GET_LOCATION_POSE_HPP_
#define THEIMC_BT_NODES__GET_LOCATION_POSE_HPP_

#include <map>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace theimc_bt_nodes
{

class GetLocationPose : public BT::SyncActionNode
{
public:
  GetLocationPose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  struct Pose
  {
    double x;
    double y;
    double yaw;
  };

  void loadLocations(const std::string & yaml_path);

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, Pose> locations_;
  std::string yaml_path_;
  std::string load_error_;
};

}  // namespace theimc_bt_nodes

#endif  // THEIMC_BT_NODES__GET_LOCATION_POSE_HPP_
