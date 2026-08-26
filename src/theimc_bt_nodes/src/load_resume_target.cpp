#include "theimc_bt_nodes/load_resume_target.hpp"

#include <string>

#include "theimc_bt_nodes/rail_progress_store.hpp"

namespace theimc_bt_nodes
{

LoadResumeTarget::LoadResumeTarget(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(xml_tag_name, config)
{
}

BT::PortsList LoadResumeTarget::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "progress_file",
      "",
      "Empty = ~/.ros/theimc_rail_progress.yaml"),
    BT::OutputPort<int>("rail_id"),
    BT::OutputPort<double>("progress_m")
  };
}

BT::NodeStatus LoadResumeTarget::tick()
{
  std::string requested_file;
  getInput("progress_file", requested_file);

  const auto path =
    resolveRailProgressFile(requested_file);

  const auto resume =
    readResumeTarget(path);

  if (!resume.pending || resume.rail_id <= 0) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("rail_id", resume.rail_id);
  setOutput("progress_m", resume.progress_m);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace theimc_bt_nodes
