#include "theimc_bt_nodes/load_resume_target.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

#include "theimc_bt_nodes/rail_progress_store.hpp"

namespace theimc_bt_nodes
{
namespace
{

std::string trim(const std::string & value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(),
    [](unsigned char c) {return std::isspace(c);});

  if (first == value.end()) {
    return {};
  }

  const auto last = std::find_if_not(
    value.rbegin(), value.rend(),
    [](unsigned char c) {return std::isspace(c);}).base();

  return std::string(first, last);
}

bool parseRailList(
  const std::string & raw,
  std::vector<int> & rails)
{
  rails.clear();

  std::stringstream stream(raw);
  std::string token;

  while (std::getline(stream, token, ',')) {
    token = trim(token);

    if (token.empty()) {
      return false;
    }

    std::size_t parsed = 0;
    int rail_id = 0;

    try {
      rail_id = std::stoi(token, &parsed);
    } catch (...) {
      return false;
    }

    if (
      parsed != token.size() ||
      rail_id < 1 ||
      rail_id > 12)
    {
      return false;
    }

    if (
      std::find(
        rails.begin(),
        rails.end(),
        rail_id) == rails.end())
    {
      rails.push_back(rail_id);
    }
  }

  return !rails.empty();
}

std::string determineResumeStage(double progress_m)
{
  if (progress_m < 0.95) {
    return "WORK_1_TO_3";
  }

  if (progress_m < 1.95) {
    return "WORK_2_TO_3";
  }

  if (progress_m < 2.95) {
    return "WORK_3_ONLY";
  }

  return "EXIT_ONLY";
}

}  // namespace

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
      "/home/jeff/theimc_robot/src/theimc_bt_nodes/config/rail_progress.yaml",
      "Rail progress YAML path"),

    BT::InputPort<std::string>(
      "selected_rails",
      "",
      "Comma-separated selected rail IDs"),

    BT::OutputPort<int>("rail_id"),
    BT::OutputPort<double>("progress_m"),

    BT::OutputPort<int>(
      "next_rail_index",
      "Index of the next selected rail after resumed rail"),

    BT::OutputPort<std::string>(
      "resume_stage",
      "WORK_1_TO_3 / WORK_2_TO_3 / WORK_3_ONLY / EXIT_ONLY")
  };
}

BT::NodeStatus LoadResumeTarget::tick()
{
  std::string requested_file;
  std::string selected_rails;

  getInput("progress_file", requested_file);
  getInput("selected_rails", selected_rails);

  const auto path =
    resolveRailProgressFile(requested_file);

  const auto resume =
    readResumeTarget(path);

  if (!resume.pending || resume.rail_id <= 0) {
    return BT::NodeStatus::FAILURE;
  }

  std::vector<int> rails;

  if (!parseRailList(selected_rails, rails)) {
    return BT::NodeStatus::FAILURE;
  }

  const auto it =
    std::find(
      rails.begin(),
      rails.end(),
      resume.rail_id);

  if (it == rails.end()) {
    return BT::NodeStatus::FAILURE;
  }

  const int resume_index =
    static_cast<int>(
      std::distance(rails.begin(), it));

  const int next_rail_index =
    resume_index + 1;

  const std::string resume_stage =
    determineResumeStage(
      resume.progress_m);

  setOutput("rail_id", resume.rail_id);
  setOutput("progress_m", resume.progress_m);
  setOutput("next_rail_index", next_rail_index);
  setOutput("resume_stage", resume_stage);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace theimc_bt_nodes