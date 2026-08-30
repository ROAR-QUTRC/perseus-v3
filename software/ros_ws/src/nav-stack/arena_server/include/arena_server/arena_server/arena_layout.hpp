/// @file arena_layout.hpp
/// @brief Shared reader for arena_layout.json.

#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

namespace arena_server {

/// @brief A named rectangle in the arena frame, centre plus size.
struct Zone {
  std::string name;
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};
  std::array<float, 3> color{0.8f, 0.8f, 0.8f};

  /// @brief Which edges to draw, from the JSON's optional "edges" list.
  ///
  /// Exists so nothing renders the arena perimeter. The excavation and obstacle
  /// zones together tile the whole bed and the construction zone's north and
  /// east sides are walls, so drawing every zone as a closed rectangle draws
  /// the arena outline - which is a priori wall geometry put on screen, exactly
  /// what guidebook 5.6.3 is about and what 5.6.4 makes you defend.
  ///
  /// Only the edges that separate one zone from another are informative anyway;
  /// the rest merely restate where the walls are. Absent from the JSON means
  /// all four, which is right for a zone that touches no wall.
  bool draw_north{true};
  bool draw_south{true};
  bool draw_east{true};
  bool draw_west{true};

  /// @brief True if any edge is drawn at all.
  bool has_edges() const {
    return draw_north || draw_south || draw_east || draw_west;
  }
};

/// @brief A fiducial marker's surveyed centre in the arena frame.
struct Marker {
  int id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// @brief The arena as both ends understand it.
///
/// Deliberately one file read by the robot and the base station alike. The base
/// station draws the arena from its own copy rather than having the robot stream
/// a map or a mesh over the link, so the only thing crossing the network is the
/// robot pose.
///
/// The obvious failure mode is the two ends reading DIFFERENT copies: the base
/// station then draws an arena the robot is not navigating in and nothing
/// reports an error. Both loaders log the resolved path and a checksum-ish
/// summary at startup so a mismatch is at least visible in the logs.
struct ArenaLayout {
  std::string frame{"map"};
  // Deliberately no arena/bed extent field. Guidebook 5.6.3 bars a priori wall
  // information - "known dimensions and location relative to the operational
  // area" is its own example - and 5.6.4 makes you prove you do not use it. See
  // the note at the top of arena_layout.json.
  std::vector<Zone> zones;
  std::map<int, Marker> markers;
  double marker_length_m{0.238};
  double spacing_m{0.4625};
  double spacing_tolerance_m{0.05};
  int dictionary_id{9};

  /// @brief Loads a layout from a JSON file.
  /// @param path Absolute path to arena_layout.json.
  /// @param[out] error Human-readable reason on failure.
  /// @return False if the file is missing, malformed, or has no zones.
  static bool load(const std::string &path, ArenaLayout &layout,
                   std::string &error);

  /// @brief One-line summary for logging, so two ends can be compared by eye.
  std::string summary() const;
};

} // namespace arena_server
