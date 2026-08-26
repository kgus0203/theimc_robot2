#pragma once

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace theimc_bt_nodes
{

struct RailProgressRecord
{
  double progress_m{0.0};
  bool completed{false};
};

struct ResumeTargetRecord
{
  bool pending{false};
  int rail_id{0};
  double progress_m{0.0};
};

inline std::string defaultRailProgressFile()
{
  const char * home = std::getenv("HOME");
  if (home && home[0] != '\0') {
    return std::string(home) + "/.ros/theimc_rail_progress.yaml";
  }
  return "/tmp/theimc_rail_progress.yaml";
}

inline std::string resolveRailProgressFile(const std::string & requested)
{
  if (requested.empty()) {
    return defaultRailProgressFile();
  }

  if (
    requested.size() >= 2 &&
    requested[0] == '~' &&
    requested[1] == '/')
  {
    const char * home = std::getenv("HOME");
    if (home && home[0] != '\0') {
      return std::string(home) + requested.substr(1);
    }
  }

  return requested;
}

inline YAML::Node loadRailProgressYaml(const std::string & path)
{
  std::ifstream input(path);

  if (!input.good()) {
    YAML::Node root;
    root["rails"] = YAML::Node(YAML::NodeType::Map);
    return root;
  }

  YAML::Node root = YAML::Load(input);

  if (!root || !root.IsMap()) {
    root = YAML::Node(YAML::NodeType::Map);
  }

  if (!root["rails"]) {
    root["rails"] = YAML::Node(YAML::NodeType::Map);
  }

  return root;
}

inline void atomicWriteRailYaml(
  const std::string & path,
  const YAML::Node & root)
{
  YAML::Emitter emitter;
  emitter << root;

  if (!emitter.good()) {
    throw std::runtime_error(
            "Failed to serialize rail progress YAML: " +
            emitter.GetLastError());
  }

  const std::string temp_path = path + ".tmp";

  {
    std::ofstream output(temp_path, std::ios::out | std::ios::trunc);

    if (!output.good()) {
      throw std::runtime_error(
              "Failed to open temp progress file: " + temp_path);
    }

    output << emitter.c_str() << '\n';
    output.flush();

    if (!output.good()) {
      throw std::runtime_error(
              "Failed to write progress file: " + temp_path);
    }
  }

  std::remove(path.c_str());

  if (std::rename(temp_path.c_str(), path.c_str()) != 0) {
    std::remove(temp_path.c_str());
    throw std::runtime_error(
            "Failed to replace progress file: " + path);
  }
}

inline RailProgressRecord readRailProgress(
  const std::string & path,
  int rail_id)
{
  RailProgressRecord result;

  const YAML::Node root = loadRailProgressYaml(path);
  const std::string rail_key = std::to_string(rail_id);
  const YAML::Node rail = root["rails"][rail_key];

  if (!rail) {
    return result;
  }

  if (rail["progress_m"]) {
    result.progress_m = rail["progress_m"].as<double>();
  }

  if (rail["completed"]) {
    result.completed = rail["completed"].as<bool>();
  }

  return result;
}

inline void writeRailProgress(
  const std::string & path,
  int rail_id,
  const RailProgressRecord & record,
  bool mark_resume)
{
  YAML::Node root = loadRailProgressYaml(path);
  const std::string rail_key = std::to_string(rail_id);

  root["rails"][rail_key]["progress_m"] =
    std::max(0.0, record.progress_m);

  root["rails"][rail_key]["completed"] =
    record.completed;

  if (mark_resume) {
    root["resume"]["pending"] = true;
    root["resume"]["rail_id"] = rail_id;
  }

  atomicWriteRailYaml(path, root);
}

inline ResumeTargetRecord readResumeTarget(
  const std::string & path)
{
  ResumeTargetRecord result;

  const YAML::Node root = loadRailProgressYaml(path);
  const YAML::Node resume = root["resume"];

  if (!resume) {
    return result;
  }

  if (resume["pending"]) {
    result.pending = resume["pending"].as<bool>();
  }

  if (!result.pending || !resume["rail_id"]) {
    result.pending = false;
    return result;
  }

  result.rail_id = resume["rail_id"].as<int>();

  if (result.rail_id <= 0) {
    result.pending = false;
    result.rail_id = 0;
    return result;
  }

  const RailProgressRecord progress =
    readRailProgress(path, result.rail_id);

  result.progress_m = progress.progress_m;

  return result;
}

inline void clearResumeTarget(
  const std::string & path)
{
  YAML::Node root = loadRailProgressYaml(path);

  root["resume"]["pending"] = false;
  root["resume"]["rail_id"] = 0;

  atomicWriteRailYaml(path, root);
}

}  // namespace theimc_bt_nodes
