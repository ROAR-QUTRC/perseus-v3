/// @file arena_layout.cpp
/// @brief Shared reader for arena_layout.json.

#include "arena_server/arena_server/arena_layout.hpp"

#include <fstream>
#include <sstream>

// Header-only, and present on the include path in this environment rather than
// via find_package -- see the note in CMakeLists.txt.
#include <nlohmann/json.hpp>

namespace arena_server {

namespace {

/// @brief Reads a rectangle, tolerating a missing colour.
Zone read_zone(const nlohmann::json &j, const std::string &fallback_name) {
  Zone z;
  z.name = j.value("name", fallback_name);
  z.x = j.at("x").get<double>();
  z.y = j.at("y").get<double>();
  z.width = j.at("width").get<double>();
  z.height = j.at("height").get<double>();
  if (j.contains("color") && j["color"].is_array() && j["color"].size() == 3) {
    for (size_t i = 0; i < 3; ++i)
      z.color[i] = j["color"][i].get<float>();
  }
  // Absent means all four, which is correct for a zone touching no wall. An
  // empty list is meaningful and distinct: draw nothing, for a zone whose only
  // boundary is already drawn by its neighbour.
  if (j.contains("edges") && j["edges"].is_array()) {
    z.draw_north = z.draw_south = z.draw_east = z.draw_west = false;
    for (const auto &e : j["edges"]) {
      const std::string s = e.get<std::string>();
      if (s == "N" || s == "north")
        z.draw_north = true;
      else if (s == "S" || s == "south")
        z.draw_south = true;
      else if (s == "E" || s == "east")
        z.draw_east = true;
      else if (s == "W" || s == "west")
        z.draw_west = true;
    }
  }
  return z;
}

} // namespace

bool ArenaLayout::load(const std::string &path, ArenaLayout &layout,
                       std::string &error) {
  std::ifstream in(path);
  if (!in) {
    error = "cannot open '" + path + "'";
    return false;
  }

  nlohmann::json j;
  try {
    in >> j;
  } catch (const std::exception &e) {
    error = "malformed JSON in '" + path + "': " + e.what();
    return false;
  }

  try {
    layout.frame = j.value("frame", std::string{"map"});

    layout.zones.clear();
    for (const auto &zj : j.at("zones")) {
      const Zone z = read_zone(zj, "unnamed");
      // A zero-size zone is almost certainly a typo rather than an intent, and
      // it would render as an invisible degenerate box either way.
      if (z.width <= 0.0 || z.height <= 0.0) {
        error = "zone '" + z.name + "' has non-positive size";
        return false;
      }
      layout.zones.push_back(z);
    }
    if (layout.zones.empty()) {
      error = "'zones' is empty in '" + path + "'";
      return false;
    }

    if (j.contains("fiducials")) {
      const auto &f = j["fiducials"];
      layout.dictionary_id = f.value("dictionary_id", 9);
      layout.marker_length_m = f.value("marker_length_m", 0.238);
      layout.spacing_m = f.value("spacing_m", 0.4625);
      layout.spacing_tolerance_m = f.value("spacing_tolerance_m", 0.05);
      layout.markers.clear();
      for (const auto &mj : f.value("markers", nlohmann::json::array())) {
        Marker m;
        m.id = mj.at("id").get<int>();
        m.x = mj.at("x").get<double>();
        m.y = mj.at("y").get<double>();
        m.z = mj.value("z", 0.0);
        layout.markers[m.id] = m;
      }
    }
  } catch (const std::exception &e) {
    error = std::string("unexpected structure in '") + path + "': " + e.what();
    return false;
  }

  return true;
}

std::string ArenaLayout::summary() const {
  // Deliberately includes the numbers, not just the counts: the failure this
  // guards against is the two ends loading different copies of the file, and
  // counts alone would match while the geometry differed.
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(3);
  os << zones.size() << " zones [";
  for (size_t i = 0; i < zones.size(); ++i) {
    if (i)
      os << " ";
    os << zones[i].name << "(" << zones[i].x << "," << zones[i].y << ","
       << zones[i].width << "x" << zones[i].height << ")";
  }
  os << "] " << markers.size() << " markers [";
  bool first = true;
  for (const auto &[id, m] : markers) {
    if (!first)
      os << " ";
    first = false;
    os << id << "(" << m.x << "," << m.y << ")";
  }
  os << "]";
  return os.str();
}

} // namespace arena_server
