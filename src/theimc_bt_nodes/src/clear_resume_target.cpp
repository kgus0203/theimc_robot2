#include "theimc_bt_nodes/clear_resume_target.hpp"

#include <string>

#include "theimc_bt_nodes/rail_progress_store.hpp"

namespace theimc_bt_nodes
{

ClearResumeTarget::ClearResumeTarget(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(xml_tag_name, config)
{
}

BT::PortsList ClearResumeTarget::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "progress_file",
      "",
      "Empty = ~/.ros/theimc_rail_progress.yaml")
  };
}

BT::NodeStatus ClearResumeTarget::tick()
{
  std::string requested_file;
  getInput("progress_file", requested_file);

  const auto path =
    resolveRailProgressFile(requested_file);

  clearResumeTarget(path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace theimc_bt_nodes
