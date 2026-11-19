#pragma once

#include <array>
#include <cctype>
#include <string>

namespace theimc_bt_nodes {
// Parse the existing /agv/axis_diagnostics String payload, without adding a JSON dependency.
// Fail closed if any axis or ready field is missing/malformed.
inline bool parse_axis_ready(const std::string &json, std::array<bool, 3> &ready) {
  std::array<bool, 3> seen{{false, false, false}};
  for (int axis = 1; axis <= 3; ++axis) {
    const std::string key = "\"axis\":" + std::to_string(axis);
    std::size_t p = json.find(key);
    if (p == std::string::npos) return false;
    const std::size_t end = json.find('}', p);
    if (end == std::string::npos) return false;
    p = json.find("\"ready\":", p + key.size());
    if (p == std::string::npos || p >= end) return false;
    p += sizeof("\"ready\":") - 1;
    while (p < end && std::isspace(static_cast<unsigned char>(json[p]))) ++p;
    if (json.compare(p, 4, "true") == 0) ready[axis - 1] = true;
    else if (json.compare(p, 5, "false") == 0) ready[axis - 1] = false;
    else return false;
    seen[axis - 1] = true;
  }
  return seen[0] && seen[1] && seen[2];
}
} // namespace theimc_bt_nodes