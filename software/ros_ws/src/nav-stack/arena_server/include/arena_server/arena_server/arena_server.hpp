/// @file arena_server.hpp
/// @brief Owns the Lunabotics arena layout and the map -> odom transform.

#pragma once

#include <array>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "arena_server/arena_server/arena_layout.hpp"
#include "interfaces/msg/detection_array.hpp"
#include "interfaces/srv/localise_in_arena.hpp"
#include "interfaces/srv/request_zone_waypoint.hpp"

namespace arena_server {

/// @brief Serves the arena layout and estimates where the rover is inside it.
///
/// The node owns one transform, map -> odom, where map is the guidebook's
/// Origin Point frame: centre of the front arena wall, +X east, +Y north. It
/// never touches odom -> base_link, which the EKF owns.
///
/// The transform is seeded from the initial_pose parameter so the layout is
/// usable from startup, and refined only when /arena/localise is called. It is
/// deliberately not re-estimated continuously: the camera is lazy, so between
/// requests it renders nothing and costs nothing.
class ArenaServer : public rclcpp::Node {
public:
  ArenaServer();

private:
  /// @brief Publishes the rover's pose in the arena frame for the base station.
  void _publish_robot_pose();
  /// @brief Seeds map -> odom from initial_pose so the layout works
  /// immediately.
  void _seed_from_initial_pose();
  /// @brief Rebroadcasts the current map -> odom.
  void _broadcast();
  /// @brief Builds and latches the zone outlines for RViz.
  void _publish_zone_markers();

  /// @brief Handles a /arena/localise request.
  void _on_localise(
      const std::shared_ptr<interfaces::srv::LocaliseInArena::Request> request,
      std::shared_ptr<interfaces::srv::LocaliseInArena::Response> response);

  /// @brief Handles a /arena/request_excavation_waypoint request.
  void _on_request_excavation_waypoint(
      const std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Request>
          request,
      std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Response>
          response);

  /// @brief Handles a /arena/request_construction_waypoint request.
  void _on_request_construction_waypoint(
      const std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Request>
          request,
      std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Response>
          response);

  /// @brief Shared implementation behind both waypoint services.
  ///
  /// Samples a grid of candidate points across the named zone (skipping any
  /// that fall inside avoid_zone_name, if given), and - with use_costmap true
  /// - rejects any whose own cell is unknown or over-cost, AND (if
  /// clearance_radius_m > 0) any that do not have a full disc of that radius
  /// around them equally clear - see _area_clear(). Among the survivors,
  /// picks the one minimising a weighted sum of costmap cost and distance
  /// from the rover - see waypoint_cost_weight/waypoint_distance_weight - or,
  /// with use_costmap false, just the closest to the rover (or the first
  /// sampled, if the rover's pose is not known either). The search still runs
  /// with use_costmap false rather than short-circuiting to the zone centre
  /// whenever avoid_zone_name is set: the centre is frequently inside the
  /// very area being avoided (e.g. construction_zone's centre sits on
  /// target_berm_area).
  /// @param zone_name Zone to search, exactly as named in arena_layout.json.
  /// @param avoid_zone_name Another zone's name to exclude candidates from
  /// (e.g. "target_berm_area" for construction_zone, so the chosen point
  /// never sits on the scoring target itself), or empty for no exclusion.
  /// @param wall_standoff_m Margin kept clear of this zone's wall-adjacent
  /// sides (see wall_standoff_m's own doc). Taken per-call rather than read
  /// off one member: excavation_zone (9.14 x 3.0 m) and construction_zone
  /// (2.05 x 3.41 m) are different enough in scale that one standoff cannot
  /// serve both - the same margin that leaves excavation_zone's 9 m span
  /// untouched can leave construction_zone's 2 m span with almost nothing
  /// (measured: 1.5 m collapses it to a 0.55 x 1.91 m strip), which is why
  /// there are two parameters, not one.
  /// @param clearance_radius_m Radius that must be entirely clear around a
  /// candidate for it to be accepted at all (see excavation_clearance_radius_m
  /// for why excavation needs this and construction does not), or 0 to only
  /// check each candidate's own cell as before. Has no effect with
  /// use_costmap false: there is no costmap to assess clearance against.
  /// @param[out] waypoint Chosen point, in the arena (map) frame. Only
  /// meaningful when this returns true.
  /// @param[out] cost Costmap value at the chosen point, or -1 if
  /// use_costmap is false.
  /// @param[out] costmap_age_s Age of the costmap scored against, or 0 if
  /// use_costmap is false.
  /// @param[out] error Human-readable reason on failure. On "no free point
  /// found", names the searched rectangle so a standoff/clearance combination
  /// that has collapsed it is visible without recomputing it by hand.
  /// @return False if either zone name is unknown, map -> odom is not
  /// established, or no acceptable, non-excluded, sufficiently clear cell was
  /// found.
  bool _find_safe_point(const std::string &zone_name,
                       const std::string &avoid_zone_name,
                       double wall_standoff_m, double clearance_radius_m,
                       geometry_msgs::msg::PoseStamped &waypoint, double &cost,
                       double &costmap_age_s, std::string &error) const;

  /// @brief Looks up a zone by name.
  /// @return nullptr if no zone in the layout has that name.
  const Zone *_find_zone(const std::string &name) const;

  /// @brief Checks that every costmap cell within radius_m of an odom-frame
  /// point is known and under waypoint_max_cost.
  ///
  /// Distance is measured directly in the costmap's own (odom) frame rather
  /// than converting each nearby cell back to map: map <-> odom is a rigid
  /// transform, so a Euclidean radius means the same thing in either frame,
  /// and skipping the per-cell conversion is what keeps this cheap enough to
  /// run for every candidate.
  /// @param costmap Grid to check against.
  /// @param ox, oy Centre point, in the costmap's own frame.
  /// @param radius_m Radius to require clear. <= 0 always returns true.
  /// @return False as soon as any cell in the disc is over-cost, or is
  /// unknown/outside the costmap's covered area and
  /// waypoint_unknown_is_free is false (its default: "cannot confirm clear"
  /// counts as not clear).
  bool _area_clear(const nav_msgs::msg::OccupancyGrid &costmap, double ox,
                   double oy, double radius_m) const;

  /// @brief Computes the rover's current pose in the arena (map) frame from
  /// the live odom -> base_link transform and the owned map -> odom.
  /// @param[out] pose Rover pose in the map frame. Only meaningful when this
  /// returns true.
  /// @return False if map -> odom is not established yet or odom -> base_link
  /// is unavailable.
  bool _robot_pose_in_map(geometry_msgs::msg::Pose &pose) const;

  /// @brief Caches the latest costmap for the next waypoint request. Runs on
  /// an executor thread, same as _on_pose et al.
  void _on_costmap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);

  /// @brief Waits for a detection message carrying at least two known markers.
  /// @param timeout How long to wait for the lazy camera pipeline to produce
  /// one.
  /// @return The message, or nullptr if none arrived in time.
  interfaces::msg::DetectionArray::ConstSharedPtr
  _await_detections(const rclcpp::Duration &timeout);

  /// @brief Fits map -> odom from observed against surveyed marker positions.
  ///
  /// Solves the 2D rigid transform (yaw and translation) that best aligns the
  /// observed marker centres to their surveyed ones. Deliberately uses marker
  /// POSITIONS only and never their orientations: a single ArUco's orientation
  /// is unreliable at these ranges, and using positions sidesteps every
  /// marker-frame convention question, which is where silent 180 degree errors
  /// come from.
  ///
  /// @param observed Marker centres in the odom frame.
  /// @param surveyed The same markers' centres in the arena frame, same order.
  /// @param[out] transform The fitted map -> odom.
  /// @param[out] residual_m RMS residual of the fit.
  /// @return False if fewer than two points, which cannot resolve yaw.
  bool _fit_map_to_odom(const std::vector<std::array<double, 2>> &observed,
                        const std::vector<std::array<double, 2>> &surveyed,
                        geometry_msgs::msg::TransformStamped &transform,
                        double &residual_m) const;

  /// @brief Checks observed marker separations against the surveyed rail
  /// spacing.
  /// @return Empty on success, else why the geometry was rejected.
  std::string
  _check_spacing(const std::vector<int> &ids,
                 const std::vector<std::array<double, 2>> &observed) const;

  // Frames
  std::string _map_frame;
  std::string _odom_frame;
  std::string _base_frame;

  // Layout, loaded from arena_layout.json rather than from ROS parameters, so
  // the base station's minimap can read exactly the same file.
  ArenaLayout _layout;

  // Localisation
  std::string _detections_topic;
  double _default_timeout_s{2.0};

  // RViz
  std::string _zones_topic;
  double _zone_wall_base_m{-0.15};
  double _zone_wall_height_m{0.75};
  double _zone_wall_thickness_m{0.04};
  double _zone_wall_alpha{0.35};
  bool _zone_labels{true};

  // Zone waypoint services
  //
  // use_costmap is the on/off switch: false skips the costmap entirely and
  // the two services just return their zone's centre, e.g. for use before
  // global_traversability is up, or to sanity-check the services against the
  // zone geometry alone.
  bool _use_costmap{true};
  std::string _costmap_topic{"/costmap"};
  // Candidate spacing when sampling a zone. Independent of the costmap's own
  // resolution (usually 0.1 m, see global_traversability's resolution_m) so
  // this stays cheap to sample even if that changes; there is no benefit to
  // a candidate grid finer than what the caller's own footprint distinguishes
  // between.
  double _waypoint_sample_resolution_m{0.2};
  // Above this costmap value (0-100) a candidate is rejected. 100 is a
  // confirmed obstacle cell (see global_traversability's _compute_final_cost);
  // below that the value decays with distance from the nearest one, so a
  // point just under 100 is still right at the rover's inscribed radius, not
  // actually safe. Default leaves real margin rather than only excluding
  // literal obstacle cells.
  double _waypoint_max_cost{50.0};
  // Weights for choosing among candidates that pass waypoint_max_cost:
  // cost_weight favours a wider margin from obstacles, distance_weight favours
  // a point closer to the rover right now. Both are normalised (cost against
  // its own 0-100 range, distance against the zone's diagonal) so the two
  // weights are comparable and default to summing to 1.
  double _waypoint_cost_weight{0.7};
  double _waypoint_distance_weight{0.3};
  // Keeps candidates off a zone's wall-adjacent edges. A zone's rectangle
  // spans right up to the arena wall on the sides Zone::draw_* marks false
  // (see arena_layout.hpp - those are exactly the undrawn, a-priori-known
  // wall sides), so without this the sampler can and does pick a point flush
  // against a wall, which is hard to manoeuvre right up against. Not applied
  // to a zone's interior (drawn) edges: stopping right at a shared zone
  // boundary is fine.
  //
  // Split in two - this one for excavation_zone, construction_wall_standoff_m
  // for construction_zone - because the two zones are not remotely the same
  // scale (9.14 x 3.0 m against 2.05 x 3.41 m): a standoff generous enough to
  // matter on the first can leave almost nothing of the second. Measured: 1.5
  // m here would be fine for excavation_zone but collapses construction_zone
  // to a 0.55 x 1.91 m strip once its one walled side per axis is inset.
  double _wall_standoff_m{0.5};
  // construction_zone's own, much smaller-scale wall_standoff_m - see above.
  double _construction_wall_standoff_m{0.3};
  // Radius that must be entirely clear around the excavation waypoint - not
  // just the point's own cell - because digging there means the rover then
  // reverses and/or drives forward on the spot, which a single clear cell
  // says nothing about: a cell can be free with an obstacle immediately
  // beside it. 0 disables this (falls back to the single-cell check), which
  // is what construction_zone gets - it is a drop-off point, not somewhere
  // the rover manoeuvres back and forth from.
  double _excavation_clearance_radius_m{1.0};
  // Whether an unmapped cell (costmap value -1, or simply outside the area
  // the costmap currently covers) is treated as free rather than rejected.
  //
  // Defaults false - matching global_traversability's own
  // treat_unknown_as_obstacle, whose doc calls that "a safe default for a
  // vehicle that cannot verify unmapped ground". A dig or drop-off point on
  // ground nobody has looked at could be sitting on a rock or crater that
  // just has not been scanned yet, so this is deliberately opt-in rather than
  // a default flipped to make waypoint requests succeed sooner against a
  // partially-built map: turning it on trades that safety margin for
  // availability, and should be a deliberate choice, e.g. for a small,
  // already-surveyed test arena, not something turned on to silence a "no
  // free point" warning without checking why coverage is missing first.
  bool _waypoint_unknown_is_free{false};
  // Zone to keep the construction waypoint out of. target_berm_area is the
  // scoring target (guidebook 5.2.2: only berm volume inside this box
  // counts) and sits almost centred inside construction_zone, so without
  // this the "safest" point construction_zone offers is frequently on top of
  // it. Empty disables the exclusion (excavation_zone has no equivalent, so
  // it is passed empty rather than reusing this parameter).
  std::string _construction_avoid_zone_name{"target_berm_area"};

  // State
  geometry_msgs::msg::TransformStamped _map_odom;
  bool _have_transform{false};
  bool _have_fiducial_fix{false};

  // Latest detections, filled by the on-demand subscription.
  interfaces::msg::DetectionArray::ConstSharedPtr _latest_detections;
  std::mutex _detections_mutex;

  // Latest costmap, filled by a permanent subscription (unlike detections,
  // there is no lazy camera to justify subscribing only per-request).
  // MultiThreadedExecutor means this is written from the subscription
  // callback and read from a service callback concurrently, hence the mutex -
  // same reasoning as _detections_mutex above.
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr _latest_costmap;
  mutable std::mutex _costmap_mutex;

  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

  rclcpp::TimerBase::SharedPtr _broadcast_timer;
  rclcpp::TimerBase::SharedPtr _zones_timer;
  rclcpp::TimerBase::SharedPtr _pose_timer;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _zones_pub;
  rclcpp::Service<interfaces::srv::LocaliseInArena>::SharedPtr _localise_srv;
  rclcpp::Service<interfaces::srv::RequestZoneWaypoint>::SharedPtr
      _excavation_waypoint_srv;
  rclcpp::Service<interfaces::srv::RequestZoneWaypoint>::SharedPtr
      _construction_waypoint_srv;
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr
      _detections_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmap_sub;
  rclcpp::CallbackGroup::SharedPtr _service_group;
};

} // namespace arena_server
