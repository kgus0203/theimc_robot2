#pragma once

#include <string>
#include "behaviortree_cpp_v3/action_node.h"

namespace theimc_bt_nodes
{

class ClearResumeTarget : public BT::SyncActionNode
{
public:
  ClearResumeTarget(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace theimc_bt_nodes
