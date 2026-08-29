/// @file arena_server.hpp
/// @brief Owns the Lunabotics arena layout and the map -> odom transform.

#pragma once

#include <array>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "interfaces/msg/detection_array.hpp"
#include "interfaces/srv/localise_in_arena.hpp"

namespace arena_server {

/// @brief Serves the arena layout and estimates where the rover is inside it.
///
/// The node owns one transform, map -> odom, where map is the guidebook's Origin
/// Point frame: centre of the front arena wall, +X east, +Y north. It never
/// touches odom -> base_link, which the EKF owns.
///
/// The transform is seeded from the initial_pose parameter so the layout is
/// usable from startup, and refined only when /arena/localise is called. It is
/// deliberately not re-estimated continuously: the camera is lazy, so between
/// requests it renders nothing and costs nothing.
class ArenaServer : public rclcpp::Node {
public:
  ArenaServer();

private:
  /// @brief A named rectangle in the arena frame, centre plus size.
  struct Zone {
    std::string name;
    double x{0.0};
    double y{0.0};
    double width{0.0};
    double height{0.0};
  };

  /// @brief A fiducial marker's surveyed centre in the arena frame.
  struct Marker {
    int id{0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  /// @brief Reads zone_names and the per-zone blocks into _zones.
  void _load_zones();
  /// @brief Reads fiducials.ids and the per-marker blocks into _markers.
  void _load_markers();
  /// @brief Seeds map -> odom from initial_pose so the layout works immediately.
  void _seed_from_initial_pose();
  /// @brief Rebroadcasts the current map -> odom.
  void _broadcast();
  /// @brief Builds and latches the zone outlines for RViz.
  void _publish_zone_markers();

  /// @brief Handles a /arena/localise request.
  void _on_localise(
      const std::shared_ptr<interfaces::srv::LocaliseInArena::Request> request,
      std::shared_ptr<interfaces::srv::LocaliseInArena::Response> response);

  /// @brief Waits for a detection message carrying at least two known markers.
  /// @param timeout How long to wait for the lazy camera pipeline to produce one.
  /// @return The message, or nullptr if none arrived in time.
  interfaces::msg::DetectionArray::ConstSharedPtr
  _await_detections(const rclcpp::Duration &timeout);

  /// @brief Fits map -> odom from observed against surveyed marker positions.
  ///
  /// Solves the 2D rigid transform (yaw and translation) that best aligns the
  /// observed marker centres to their surveyed ones. Deliberately uses marker
  /// POSITIONS only and never their orientations: a single ArUco's orientation is
  /// unreliable at these ranges, and using positions sidesteps every marker-frame
  /// convention question, which is where silent 180 degree errors come from.
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

  /// @brief Checks observed marker separations against the surveyed rail spacing.
  /// @return Empty on success, else why the geometry was rejected.
  std::string
  _check_spacing(const std::vector<int> &ids,
                 const std::vector<std::array<double, 2>> &observed) const;

  // Frames
  std::string _map_frame;
  std::string _odom_frame;
  std::string _base_frame;

  // Layout
  std::vector<Zone> _zones;
  std::map<int, Marker> _markers;

  // Fiducial checks
  double _spacing_m{0.4625};
  double _spacing_tolerance_m{0.05};

  // Localisation
  std::string _detections_topic;
  double _default_timeout_s{2.0};

  // RViz
  std::string _zones_topic;
  double _zone_marker_height_m{0.05};
  double _zone_line_width_m{0.03};
  bool _zone_labels{true};

  // State
  geometry_msgs::msg::TransformStamped _map_odom;
  bool _have_transform{false};
  bool _have_fiducial_fix{false};

  // Latest detections, filled by the on-demand subscription.
  interfaces::msg::DetectionArray::ConstSharedPtr _latest_detections;
  std::mutex _detections_mutex;

  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

  rclcpp::TimerBase::SharedPtr _broadcast_timer;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _zones_pub;
  rclcpp::Service<interfaces::srv::LocaliseInArena>::SharedPtr _localise_srv;
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr
      _detections_sub;
  rclcpp::CallbackGroup::SharedPtr _service_group;
};

} // namespace arena_server
