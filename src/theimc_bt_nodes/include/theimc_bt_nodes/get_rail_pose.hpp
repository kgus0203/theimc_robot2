#ifndef THEIMC_BT_NODES__GET_RAIL_POSE_HPP_
#define THEIMC_BT_NODES__GET_RAIL_POSE_HPP_

#include <map>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace theimc_bt_nodes
{

class GetRailPose : public BT::SyncActionNode
{
public:
  GetRailPose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  struct RailPose
  {
    double x;
    double y;
    double yaw;
    double enter_speed;
  };

  void loadRailPoses(const std::string & yaml_path);

  rclcpp::Node::SharedPtr node_;
  std::map<int, RailPose> rail_poses_;
  std::string yaml_path_;
  double default_enter_speed_{0.20};
  std::string load_error_;
};

}  // namespace theimc_bt_nodes

#endif  // THEIMC_BT_NODES__GET_RAIL_POSE_HPP_

