/// @file arena_server.cpp
/// @brief Implementation of the arena layout / localisation server.

#include "arena_server/arena_server/arena_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace arena_server {

ArenaServer::ArenaServer() : Node("arena_server") {
  _map_frame = declare_parameter<std::string>("map_frame", "map");
  _odom_frame = declare_parameter<std::string>("odom_frame", "odom");
  _base_frame = declare_parameter<std::string>("base_frame", "base_link");

  _detections_topic = declare_parameter<std::string>(
      "detections_topic", "/vision/aruco/detections");
  _default_timeout_s =
      declare_parameter<double>("default_localise_timeout_s", 2.0);

  _zones_topic = declare_parameter<std::string>("zones_topic", "/arena/zones");
  _zone_wall_base_m = declare_parameter<double>("zone_wall_base_m", -0.15);
  _zone_wall_height_m = declare_parameter<double>("zone_wall_height_m", 0.75);
  _zone_wall_thickness_m =
      declare_parameter<double>("zone_wall_thickness_m", 0.04);
  _zone_wall_alpha = declare_parameter<double>("zone_wall_alpha", 0.35);
  _zone_labels = declare_parameter<bool>("zone_labels", true);

  _use_costmap = declare_parameter<bool>("use_costmap", true);
  _costmap_topic =
      declare_parameter<std::string>("costmap_topic", "/costmap");
  _waypoint_sample_resolution_m =
      declare_parameter<double>("waypoint_sample_resolution_m", 0.2);
  _waypoint_max_cost = declare_parameter<double>("waypoint_max_cost", 50.0);
  _waypoint_cost_weight =
      declare_parameter<double>("waypoint_cost_weight", 0.7);
  _waypoint_distance_weight =
      declare_parameter<double>("waypoint_distance_weight", 0.3);

  // The layout is JSON, not ROS parameters, so the base station's minimap can
  // read byte-for-byte the same file. Both ends log summary() at startup: the
  // failure this guards against is the two reading different copies, which
  // otherwise shows up as a base station drawing an arena the robot is not in,
  // with nothing reporting an error.
  const auto layout_file = declare_parameter<std::string>("layout_file", "");
  std::string layout_error;
  if (!ArenaLayout::load(layout_file, _layout, layout_error)) {
    RCLCPP_FATAL(get_logger(), "cannot load arena layout: %s",
                 layout_error.c_str());
    throw std::runtime_error("arena layout: " + layout_error);
  }
  RCLCPP_INFO(get_logger(), "layout from %s", layout_file.c_str());
  RCLCPP_INFO(get_logger(), "layout is %s", _layout.summary().c_str());

  _tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
  _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (declare_parameter<bool>("publish_initial_pose", true)) {
    _seed_from_initial_pose();
  }

  // Transient local so a correctly-configured subscriber joining late gets the
  // zones immediately. That is not enough on its own: RViz's MarkerArray
  // display defaults to VOLATILE durability, and a volatile subscriber never
  // receives a message published before it connected - so with a publish-once
  // latched topic the zones simply never appear, which is what happened.
  // Measured against the running node: a VOLATILE subscriber got nothing, a
  // TRANSIENT_LOCAL one got all 10 markers.
  //
  // Hence the periodic republish below as well. Ten markers at 1 Hz is nothing,
  // and it means the display works without anyone having to know to change its
  // QoS. Set zone_republish_hz to 0 to publish once only.
  const auto latched = rclcpp::QoS(1).transient_local();
  _zones_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      _zones_topic, latched);
  _publish_zone_markers();

  const double zones_hz = declare_parameter<double>("zone_republish_hz", 1.0);
  if (zones_hz > 0.0) {
    _zones_timer =
        create_wall_timer(std::chrono::duration<double>(1.0 / zones_hz),
                          [this]() { _publish_zone_markers(); });
  }

  // The service blocks waiting for camera frames, so it needs its own reentrant
  // group: on the default group it would block the executor and the very
  // subscription it is waiting on would never be serviced.
  _service_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _localise_srv = create_service<interfaces::srv::LocaliseInArena>(
      "/arena/localise",
      std::bind(&ArenaServer::_on_localise, this, std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS(), _service_group);

  // Neither waypoint service blocks, so both are fine on the node's default
  // callback group - unlike /arena/localise they don't need the reentrant one.
  _excavation_waypoint_srv = create_service<interfaces::srv::RequestZoneWaypoint>(
      "/arena/request_excavation_waypoint",
      std::bind(&ArenaServer::_on_request_excavation_waypoint, this,
                std::placeholders::_1, std::placeholders::_2));
  _construction_waypoint_srv =
      create_service<interfaces::srv::RequestZoneWaypoint>(
          "/arena/request_construction_waypoint",
          std::bind(&ArenaServer::_on_request_construction_waypoint, this,
                    std::placeholders::_1, std::placeholders::_2));

  // Permanent subscription, not on-demand like _detections_sub: there is no
  // lazy camera here to justify tearing it down between requests, and
  // global_traversability only republishes every update_period_s (5 s by
  // default), so a fresh subscribe-per-request could easily wait a full
  // period for the first message.
  if (_use_costmap) {
    _costmap_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
        _costmap_topic, rclcpp::QoS(1).transient_local(),
        std::bind(&ArenaServer::_on_costmap, this, std::placeholders::_1));
  }

  // The one thing that has to cross the network. The base station draws the
  // arena from its own copy of the JSON, so this small pose is all it needs -
  // no costmap, no mesh, no map server. Kept deliberately low rate for the same
  // reason; a minimap does not need 20 Hz.
  _pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("robot_pose_topic", "/arena/robot_pose"),
      rclcpp::QoS(5));
  const double pose_hz = declare_parameter<double>("robot_pose_rate_hz", 5.0);
  if (pose_hz > 0.0) {
    _pose_timer =
        create_wall_timer(std::chrono::duration<double>(1.0 / pose_hz),
                          std::bind(&ArenaServer::_publish_robot_pose, this));
  }

  const double rate = declare_parameter<double>("broadcast_rate_hz", 20.0);
  _broadcast_timer = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
      std::bind(&ArenaServer::_broadcast, this));

  RCLCPP_INFO(get_logger(),
              "arena_server up: %zu zones, %zu markers, publishing %s -> %s",
              _layout.zones.size(), _layout.markers.size(), _map_frame.c_str(),
              _odom_frame.c_str());
}

void ArenaServer::_seed_from_initial_pose() {
  const double x = declare_parameter<double>("initial_pose.x", 0.0);
  const double y = declare_parameter<double>("initial_pose.y", 0.0);
  const double yaw = declare_parameter<double>("initial_pose.yaw", 0.0);

  // The parameter states where the rover is in map. What we publish is
  // map -> odom, so odom -> base_link has to come out of it. At startup that is
  // usually identity, but not if this node is restarted mid-run, so look it up
  // and fall back rather than assuming.
  double ox = 0.0, oy = 0.0, oyaw = 0.0;
  try {
    const auto odom_base = _tf_buffer->lookupTransform(
        _odom_frame, _base_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.5));
    ox = odom_base.transform.translation.x;
    oy = odom_base.transform.translation.y;
    oyaw = tf2::getYaw(odom_base.transform.rotation);
    RCLCPP_INFO(get_logger(),
                "seeding against a live %s -> %s (%.3f, %.3f, %.3f rad)",
                _odom_frame.c_str(), _base_frame.c_str(), ox, oy, oyaw);
  } catch (const tf2::TransformException &e) {
    RCLCPP_INFO(get_logger(),
                "no %s -> %s yet (%s); assuming odom starts at the rover",
                _odom_frame.c_str(), _base_frame.c_str(), e.what());
  }

  // map->odom = map->base * (odom->base)^-1, in 2D.
  const double c = std::cos(yaw - oyaw), s = std::sin(yaw - oyaw);
  _map_odom.header.frame_id = _map_frame;
  _map_odom.child_frame_id = _odom_frame;
  _map_odom.transform.translation.x = x - (c * ox - s * oy);
  _map_odom.transform.translation.y = y - (s * ox + c * oy);
  _map_odom.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw - oyaw);
  _map_odom.transform.rotation = tf2::toMsg(q);
  _have_transform = true;

  RCLCPP_INFO(get_logger(),
              "seeded from initial_pose (%.3f, %.3f, %.4f rad). This is an "
              "assumption - the guidebook randomises the start pose, so call "
              "/arena/localise before trusting it.",
              x, y, yaw);
}

bool ArenaServer::_robot_pose_in_map(geometry_msgs::msg::Pose &pose) const {
  if (!_have_transform)
    return false;
  geometry_msgs::msg::TransformStamped odom_base;
  try {
    odom_base = _tf_buffer->lookupTransform(_odom_frame, _base_frame,
                                            tf2::TimePointZero,
                                            tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException &) {
    // Nothing publishing odom -> base_link yet.
    return false;
  }

  const double yaw = tf2::getYaw(_map_odom.transform.rotation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  pose.position.x = _map_odom.transform.translation.x +
                    c * odom_base.transform.translation.x -
                    s * odom_base.transform.translation.y;
  pose.position.y = _map_odom.transform.translation.y +
                    s * odom_base.transform.translation.x +
                    c * odom_base.transform.translation.y;
  pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw + tf2::getYaw(odom_base.transform.rotation));
  pose.orientation = tf2::toMsg(q);
  return true;
}

void ArenaServer::_publish_robot_pose() {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = _map_frame;
  pose.header.stamp = now();
  // Silent on failure, on purpose: this runs on a timer, and warning every
  // cycle before the EKF/localise fix is up is pure noise.
  if (!_robot_pose_in_map(pose.pose))
    return;
  _pose_pub->publish(pose);
}

void ArenaServer::_broadcast() {
  if (!_have_transform)
    return;
  _map_odom.header.stamp = now();
  _tf_broadcaster->sendTransform(_map_odom);
}

void ArenaServer::_publish_zone_markers() {
  visualization_msgs::msg::MarkerArray array;
  int id = 0;

  for (const auto &z : _layout.zones) {
    const auto &rgb = z.color;

    const double hx = z.width * 0.5, hy = z.height * 0.5;
    const double t = _zone_wall_thickness_m;
    const double h = _zone_wall_height_m;
    const double cz = _zone_wall_base_m + h * 0.5;

    // Four upright slabs rather than a flat outline on the ground. A LINE_STRIP
    // at grade disappears the moment a 3D terrain mesh is displayed over it -
    // it is coplanar with the thing hiding it. Walls stand above the mesh, and
    // the alpha lets you see the arena through them.
    //
    // Each edge is a thin CUBE. The along-edge dimension carries + t so the
    // corners meet instead of leaving four notches.
    // Only the edges the layout marks as interior. Edges that coincide with an
    // arena wall are skipped, so nothing here renders the arena perimeter -
    // see the note on Zone::draw_north in arena_layout.hpp.
    struct Edge {
      double cx, cy, sx, sy;
      bool draw;
    };
    const Edge edges[4] = {{z.x, z.y - hy, 2.0 * hx + t, t, z.draw_south},
                           {z.x, z.y + hy, 2.0 * hx + t, t, z.draw_north},
                           {z.x - hx, z.y, t, 2.0 * hy + t, z.draw_west},
                           {z.x + hx, z.y, t, 2.0 * hy + t, z.draw_east}};

    for (const auto &e : edges) {
      if (!e.draw)
        continue;
      visualization_msgs::msg::Marker wall;
      wall.header.frame_id = _map_frame;
      wall.header.stamp = now();
      wall.ns = "arena_zones";
      wall.id = id++;
      wall.type = visualization_msgs::msg::Marker::CUBE;
      wall.action = visualization_msgs::msg::Marker::ADD;
      wall.pose.position.x = e.cx;
      wall.pose.position.y = e.cy;
      wall.pose.position.z = cz;
      wall.pose.orientation.w = 1.0;
      wall.scale.x = e.sx;
      wall.scale.y = e.sy;
      wall.scale.z = h;
      wall.color.r = rgb[0];
      wall.color.g = rgb[1];
      wall.color.b = rgb[2];
      wall.color.a = static_cast<float>(_zone_wall_alpha);
      array.markers.push_back(wall);
    }

    if (_zone_labels && z.has_edges()) {
      visualization_msgs::msg::Marker label;
      label.header.frame_id = _map_frame;
      label.header.stamp = now();
      label.ns = "arena_zone_labels";
      label.id = id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = z.x;
      label.pose.position.y = z.y;
      // Above the walls, so the text is not lost inside a translucent slab.
      label.pose.position.z = _zone_wall_base_m + h + 0.25;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.22;
      label.color.r = rgb[0];
      label.color.g = rgb[1];
      label.color.b = rgb[2];
      label.color.a = 1.0f;
      label.text = z.name;
      array.markers.push_back(label);
    }
  }

  _zones_pub->publish(array);
  // Logged once: this is on a repeating timer, so INFO every cycle would be
  // noise.
  RCLCPP_INFO_ONCE(get_logger(),
                   "publishing %zu zone markers on %s (transient local, "
                   "republished so volatile subscribers such as RViz see them)",
                   array.markers.size(), _zones_topic.c_str());
}

std::string ArenaServer::_check_spacing(
    const std::vector<int> &ids,
    const std::vector<std::array<double, 2>> &observed) const {
  for (size_t i = 0; i < ids.size(); ++i) {
    for (size_t j = i + 1; j < ids.size(); ++j) {
      const auto a = _layout.markers.find(ids[i]);
      const auto b = _layout.markers.find(ids[j]);
      if (a == _layout.markers.end() || b == _layout.markers.end())
        continue;
      const double expected =
          std::hypot(a->second.x - b->second.x, a->second.y - b->second.y);
      const double measured = std::hypot(observed[i][0] - observed[j][0],
                                         observed[i][1] - observed[j][1]);
      if (std::fabs(measured - expected) > _layout.spacing_tolerance_m) {
        return "markers " + std::to_string(ids[i]) + " and " +
               std::to_string(ids[j]) + " are " + std::to_string(measured) +
               " m apart but should be " + std::to_string(expected) +
               " m; rejecting as a false detection";
      }
    }
  }
  return {};
}

bool ArenaServer::_fit_map_to_odom(
    const std::vector<std::array<double, 2>> &observed,
    const std::vector<std::array<double, 2>> &surveyed,
    geometry_msgs::msg::TransformStamped &transform, double &residual_m) const {
  // Two points is the minimum that resolves yaw. One would fix translation only
  // and leave the arena free to rotate about it, which is exactly the failure
  // that makes a single-marker fix untrustworthy.
  if (observed.size() < 2 || observed.size() != surveyed.size())
    return false;

  const size_t n = observed.size();
  double ocx = 0.0, ocy = 0.0, scx = 0.0, scy = 0.0;
  for (size_t i = 0; i < n; ++i) {
    ocx += observed[i][0];
    ocy += observed[i][1];
    scx += surveyed[i][0];
    scy += surveyed[i][1];
  }
  ocx /= static_cast<double>(n);
  ocy /= static_cast<double>(n);
  scx /= static_cast<double>(n);
  scy /= static_cast<double>(n);

  // Closed-form 2D Kabsch: the yaw that best aligns the centred point sets.
  double sxy = 0.0, cxy = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double ox = observed[i][0] - ocx, oy = observed[i][1] - ocy;
    const double sx = surveyed[i][0] - scx, sy = surveyed[i][1] - scy;
    cxy += ox * sx + oy * sy;
    sxy += ox * sy - oy * sx;
  }
  const double yaw = std::atan2(sxy, cxy);
  const double c = std::cos(yaw), s = std::sin(yaw);

  double sum_sq = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double ox = observed[i][0] - ocx, oy = observed[i][1] - ocy;
    const double px = c * ox - s * oy + scx;
    const double py = s * ox + c * oy + scy;
    sum_sq +=
        std::pow(px - surveyed[i][0], 2) + std::pow(py - surveyed[i][1], 2);
  }
  residual_m = std::sqrt(sum_sq / static_cast<double>(n));

  transform.header.frame_id = _map_frame;
  transform.child_frame_id = _odom_frame;
  transform.transform.translation.x = scx - (c * ocx - s * ocy);
  transform.transform.translation.y = scy - (s * ocx + c * ocy);
  transform.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  transform.transform.rotation = tf2::toMsg(q);
  return true;
}

interfaces::msg::DetectionArray::ConstSharedPtr
ArenaServer::_await_detections(const rclcpp::Duration &timeout) {
  {
    std::lock_guard<std::mutex> lock(_detections_mutex);
    _latest_detections.reset();
  }

  // Subscribe only for the duration of the request. The camera is lazy, so this
  // subscription is what makes it render at all - and dropping it afterwards is
  // what makes it stop.
  rclcpp::SubscriptionOptions options;
  options.callback_group = _service_group;
  _detections_sub = create_subscription<interfaces::msg::DetectionArray>(
      _detections_topic, rclcpp::SensorDataQoS(),
      [this](interfaces::msg::DetectionArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(_detections_mutex);
        _latest_detections = msg;
      },
      options);

  const auto deadline = now() + timeout;
  interfaces::msg::DetectionArray::ConstSharedPtr found;
  while (rclcpp::ok() && now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(_detections_mutex);
      if (_latest_detections) {
        // Count how many of the markers in this frame we have surveyed.
        size_t known = 0;
        for (const auto &d : _latest_detections->detections) {
          if (d.has_pose && _layout.markers.count(d.id))
            ++known;
        }
        if (known >= 2) {
          found = _latest_detections;
          break;
        }
      }
    }
    rclcpp::sleep_for(20ms);
  }

  _detections_sub.reset();
  return found;
}

void ArenaServer::_on_localise(
    const std::shared_ptr<interfaces::srv::LocaliseInArena::Request> request,
    std::shared_ptr<interfaces::srv::LocaliseInArena::Response> response) {
  response->success = false;

  const double timeout_s =
      request->timeout_s > 0.0 ? request->timeout_s : _default_timeout_s;
  const auto detections =
      _await_detections(rclcpp::Duration::from_seconds(timeout_s));

  if (!detections) {
    response->message =
        "no frame with at least two known markers within " +
        std::to_string(timeout_s) +
        " s. Two are required: one marker fixes translation but leaves the "
        "arena free to rotate about it. Note the rail is only visible from the "
        "back of the starting zone - closer than ~0.8 m the markers fall out "
        "of "
        "the camera's view entirely.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Detection poses are in the camera optical frame; move them into odom so the
  // fit is against a frame that does not move with the robot.
  geometry_msgs::msg::TransformStamped odom_cam;
  try {
    odom_cam = _tf_buffer->lookupTransform(
        _odom_frame, detections->header.frame_id, detections->header.stamp,
        tf2::durationFromSec(0.3));
  } catch (const tf2::TransformException &e) {
    response->message = std::string("cannot transform ") +
                        detections->header.frame_id + " -> " + _odom_frame +
                        ": " + e.what();
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  std::vector<int> ids;
  std::vector<std::array<double, 2>> observed, surveyed;
  for (const auto &d : detections->detections) {
    if (!d.has_pose)
      continue;
    const auto known = _layout.markers.find(d.id);
    if (known == _layout.markers.end())
      continue; // whitelist: ignore stray ids

    geometry_msgs::msg::PoseStamped in, out;
    in.header = detections->header;
    in.pose = d.pose;
    tf2::doTransform(in, out, odom_cam);

    ids.push_back(d.id);
    observed.push_back({out.pose.position.x, out.pose.position.y});
    surveyed.push_back({known->second.x, known->second.y});
  }

  if (ids.size() < 2) {
    response->message = "only " + std::to_string(ids.size()) +
                        " known marker(s) had a usable pose; need two";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  const std::string geometry_error = _check_spacing(ids, observed);
  if (!geometry_error.empty()) {
    response->message = geometry_error;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  geometry_msgs::msg::TransformStamped fitted;
  double residual = 0.0;
  if (!_fit_map_to_odom(observed, surveyed, fitted, residual)) {
    response->message = "rigid fit failed";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  _map_odom = fitted;
  _have_transform = true;
  _have_fiducial_fix = true;
  _broadcast();

  // Report where the rover now is, which is what the caller actually asked for.
  try {
    const auto odom_base = _tf_buffer->lookupTransform(
        _odom_frame, _base_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.3));
    const double yaw = tf2::getYaw(fitted.transform.rotation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    response->pose.header.frame_id = _map_frame;
    response->pose.header.stamp = now();
    response->pose.pose.position.x = fitted.transform.translation.x +
                                     c * odom_base.transform.translation.x -
                                     s * odom_base.transform.translation.y;
    response->pose.pose.position.y = fitted.transform.translation.y +
                                     s * odom_base.transform.translation.x +
                                     c * odom_base.transform.translation.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw + tf2::getYaw(odom_base.transform.rotation));
    response->pose.pose.orientation = tf2::toMsg(q);
  } catch (const tf2::TransformException &e) {
    response->message = std::string("fix accepted but ") + _odom_frame +
                        " -> " + _base_frame + " is unavailable: " + e.what();
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->success = true;
  response->marker_ids = ids;
  response->residual_m = residual;
  response->message = "fix from " + std::to_string(ids.size()) +
                      " markers, residual " + std::to_string(residual) + " m";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void ArenaServer::_on_costmap(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(_costmap_mutex);
  _latest_costmap = message;
}

bool ArenaServer::_find_safe_point(const std::string &zone_name,
                                   geometry_msgs::msg::PoseStamped &waypoint,
                                   double &cost, double &costmap_age_s,
                                   std::string &error) const {
  cost = -1.0;
  costmap_age_s = 0.0;

  const Zone *zone = nullptr;
  for (const auto &z : _layout.zones) {
    if (z.name == zone_name) {
      zone = &z;
      break;
    }
  }
  if (!zone) {
    error = "no zone named '" + zone_name + "' in the loaded layout";
    return false;
  }

  if (!_have_transform) {
    error = _map_frame + " -> " + _odom_frame + " not established yet";
    return false;
  }

  waypoint.header.frame_id = _map_frame;
  waypoint.header.stamp = now();
  waypoint.pose.orientation.w = 1.0;

  if (!_use_costmap) {
    waypoint.pose.position.x = zone->x;
    waypoint.pose.position.y = zone->y;
    return true;
  }

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(_costmap_mutex);
    costmap = _latest_costmap;
  }
  if (!costmap) {
    error = "use_costmap is true but nothing has been received yet on " +
           _costmap_topic;
    return false;
  }
  if (costmap->info.resolution <= 0.0 || costmap->info.width == 0 ||
      costmap->info.height == 0) {
    error = "received costmap has no usable grid (zero resolution or size)";
    return false;
  }

  geometry_msgs::msg::Pose robot_pose;
  const bool have_robot_pose = _robot_pose_in_map(robot_pose);

  // map -> odom, the inverse of the rotation _robot_pose_in_map applies going
  // odom -> map, needed here because the costmap's cells are indexed in odom.
  const double yaw = tf2::getYaw(_map_odom.transform.rotation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double tx = _map_odom.transform.translation.x;
  const double ty = _map_odom.transform.translation.y;

  const double hx = zone->width * 0.5, hy = zone->height * 0.5;
  const double step = _waypoint_sample_resolution_m > 0.0
                          ? _waypoint_sample_resolution_m
                          : costmap->info.resolution;
  const double diagonal_m = std::hypot(zone->width, zone->height);

  bool found = false;
  double best_score = std::numeric_limits<double>::infinity();
  double best_x = 0.0, best_y = 0.0, best_cost = 0.0;

  for (double mx = zone->x - hx; mx <= zone->x + hx; mx += step) {
    for (double my = zone->y - hy; my <= zone->y + hy; my += step) {
      const double dx = mx - tx, dy = my - ty;
      const double ox = c * dx + s * dy;
      const double oy = -s * dx + c * dy;

      const int col = static_cast<int>(std::floor(
          (ox - costmap->info.origin.position.x) / costmap->info.resolution));
      const int row = static_cast<int>(std::floor(
          (oy - costmap->info.origin.position.y) / costmap->info.resolution));
      if (col < 0 || row < 0 ||
          col >= static_cast<int>(costmap->info.width) ||
          row >= static_cast<int>(costmap->info.height))
        continue; // outside what the costmap currently covers

      const int8_t value =
          costmap->data[static_cast<size_t>(row) * costmap->info.width +
                       static_cast<size_t>(col)];
      if (value < 0 || static_cast<double>(value) > _waypoint_max_cost)
        continue; // unknown, or over the safety margin

      double score = _waypoint_cost_weight * (static_cast<double>(value) / 100.0);
      if (have_robot_pose) {
        const double dist =
            std::hypot(mx - robot_pose.position.x, my - robot_pose.position.y);
        score += _waypoint_distance_weight *
                (diagonal_m > 0.0 ? dist / diagonal_m : dist);
      }

      if (score < best_score) {
        best_score = score;
        best_x = mx;
        best_y = my;
        best_cost = static_cast<double>(value);
        found = true;
      }
    }
  }

  if (!found) {
    error = "no cell under waypoint_max_cost (" +
           std::to_string(_waypoint_max_cost) + ") found in zone '" +
           zone_name +
           "'; the costmap may not cover it yet or it is fully obstructed";
    return false;
  }

  waypoint.pose.position.x = best_x;
  waypoint.pose.position.y = best_y;
  cost = best_cost;
  costmap_age_s = (now() - costmap->header.stamp).seconds();
  return true;
}

void ArenaServer::_on_request_excavation_waypoint(
    const std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Request>,
    std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Response>
        response) {
  std::string error;
  response->success =
      _find_safe_point("excavation_zone", response->waypoint,
                       response->cost, response->costmap_age_s, error);
  response->message = error;
  if (!response->success)
    RCLCPP_WARN(get_logger(), "%s", error.c_str());
}

void ArenaServer::_on_request_construction_waypoint(
    const std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Request>,
    std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Response>
        response) {
  std::string error;
  response->success =
      _find_safe_point("construction_zone", response->waypoint,
                       response->cost, response->costmap_age_s, error);
  response->message = error;
  if (!response->success)
    RCLCPP_WARN(get_logger(), "%s", error.c_str());
}

} // namespace arena_server
