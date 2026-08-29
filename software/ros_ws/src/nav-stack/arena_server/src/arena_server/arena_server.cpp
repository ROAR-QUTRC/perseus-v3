/// @file arena_server.cpp
/// @brief Implementation of the arena layout / localisation server.

#include "arena_server/arena_server/arena_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

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
  _default_timeout_s = declare_parameter<double>("default_localise_timeout_s", 2.0);
  _spacing_m = declare_parameter<double>("fiducials.spacing_m", 0.4625);
  _spacing_tolerance_m =
      declare_parameter<double>("fiducials.spacing_tolerance_m", 0.05);

  _zones_topic = declare_parameter<std::string>("zones_topic", "/arena/zones");
  _zone_marker_height_m = declare_parameter<double>("zone_marker_height_m", 0.05);
  _zone_line_width_m = declare_parameter<double>("zone_line_width_m", 0.03);
  _zone_labels = declare_parameter<bool>("zone_labels", true);

  _load_zones();
  _load_markers();

  _tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
  _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (declare_parameter<bool>("publish_initial_pose", true)) {
    _seed_from_initial_pose();
  }

  // Latched so an RViz joining later gets the zones immediately rather than
  // waiting for a redraw that never comes - they are published once.
  const auto latched = rclcpp::QoS(1).transient_local();
  _zones_pub = create_publisher<visualization_msgs::msg::MarkerArray>(_zones_topic,
                                                                     latched);
  _publish_zone_markers();

  // The service blocks waiting for camera frames, so it needs its own reentrant
  // group: on the default group it would block the executor and the very
  // subscription it is waiting on would never be serviced.
  _service_group =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _localise_srv = create_service<interfaces::srv::LocaliseInArena>(
      "/arena/localise",
      std::bind(&ArenaServer::_on_localise, this, std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS(), _service_group);

  const double rate = declare_parameter<double>("broadcast_rate_hz", 20.0);
  _broadcast_timer = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
      std::bind(&ArenaServer::_broadcast, this));

  RCLCPP_INFO(get_logger(),
              "arena_server up: %zu zones, %zu markers, publishing %s -> %s",
              _zones.size(), _markers.size(), _map_frame.c_str(),
              _odom_frame.c_str());
}

void ArenaServer::_load_zones() {
  // The default is spelled out rather than left as {}: with a bare initialiser
  // list the compiler picks the ParameterDescriptor overload instead, which
  // declares the parameter with no default at all and throws rather than
  // returning empty if the layout file ever omits zone_names.
  const auto names = declare_parameter("zone_names", std::vector<std::string>{});
  for (const auto &name : names) {
    Zone z;
    z.name = name;
    z.x = declare_parameter<double>(name + ".x", 0.0);
    z.y = declare_parameter<double>(name + ".y", 0.0);
    z.width = declare_parameter<double>(name + ".width", 0.0);
    z.height = declare_parameter<double>(name + ".height", 0.0);
    if (z.width <= 0.0 || z.height <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "zone '%s' has non-positive size (%.3f x %.3f); skipping",
                  name.c_str(), z.width, z.height);
      continue;
    }
    RCLCPP_INFO(get_logger(), "zone '%s': centre (%.3f, %.3f) size %.2f x %.2f",
                z.name.c_str(), z.x, z.y, z.width, z.height);
    _zones.push_back(z);
  }
}

void ArenaServer::_load_markers() {
  const auto ids = declare_parameter("fiducials.ids", std::vector<int64_t>{});
  for (const auto id64 : ids) {
    const int id = static_cast<int>(id64);
    const std::string key = "marker_" + std::to_string(id);
    Marker m;
    m.id = id;
    m.x = declare_parameter<double>(key + ".x", 0.0);
    m.y = declare_parameter<double>(key + ".y", 0.0);
    m.z = declare_parameter<double>(key + ".z", 0.0);
    _markers[id] = m;
    RCLCPP_INFO(get_logger(), "marker %d at (%.4f, %.4f, %.3f)", id, m.x, m.y,
                m.z);
  }
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
        _odom_frame, _base_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
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

void ArenaServer::_broadcast() {
  if (!_have_transform) return;
  _map_odom.header.stamp = now();
  _tf_broadcaster->sendTransform(_map_odom);
}


void ArenaServer::_publish_zone_markers() {
  visualization_msgs::msg::MarkerArray array;
  int id = 0;

  // One colour per zone, matching the sim's lunabotics_zones outlines so the two
  // views agree at a glance.
  const std::map<std::string, std::array<float, 3>> colours = {
      {"starting_zone", {0.10f, 0.70f, 0.20f}},
      {"excavation_zone", {0.55f, 0.55f, 0.55f}},
      {"obstacle_zone", {0.20f, 0.45f, 0.85f}},
      {"construction_zone", {0.95f, 0.55f, 0.05f}},
      {"target_berm_area", {0.85f, 0.05f, 0.05f}}};

  for (const auto &z : _zones) {
    visualization_msgs::msg::Marker outline;
    outline.header.frame_id = _map_frame;
    outline.header.stamp = now();
    outline.ns = "arena_zones";
    outline.id = id++;
    outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
    outline.action = visualization_msgs::msg::Marker::ADD;
    outline.scale.x = _zone_line_width_m;
    outline.pose.orientation.w = 1.0;

    const auto it = colours.find(z.name);
    const auto rgb =
        it != colours.end() ? it->second : std::array<float, 3>{0.8f, 0.8f, 0.8f};
    outline.color.r = rgb[0];
    outline.color.g = rgb[1];
    outline.color.b = rgb[2];
    outline.color.a = 0.9f;

    const double hx = z.width * 0.5, hy = z.height * 0.5;
    const double zh = _zone_marker_height_m;
    // Closed rectangle: five points so the last edge is drawn.
    const double corners[5][2] = {{z.x - hx, z.y - hy},
                                  {z.x + hx, z.y - hy},
                                  {z.x + hx, z.y + hy},
                                  {z.x - hx, z.y + hy},
                                  {z.x - hx, z.y - hy}};
    for (const auto &c : corners) {
      geometry_msgs::msg::Point p;
      p.x = c[0];
      p.y = c[1];
      p.z = zh;
      outline.points.push_back(p);
    }
    array.markers.push_back(outline);

    if (_zone_labels) {
      visualization_msgs::msg::Marker label;
      label.header = outline.header;
      label.ns = "arena_zone_labels";
      label.id = id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = z.x;
      label.pose.position.y = z.y;
      label.pose.position.z = zh + 0.25;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.20;
      label.color = outline.color;
      label.color.a = 1.0f;
      label.text = z.name;
      array.markers.push_back(label);
    }
  }

  _zones_pub->publish(array);
  RCLCPP_INFO(get_logger(), "published %zu zone markers on %s (latched)",
              array.markers.size(), _zones_topic.c_str());
}

std::string ArenaServer::_check_spacing(
    const std::vector<int> &ids,
    const std::vector<std::array<double, 2>> &observed) const {
  for (size_t i = 0; i < ids.size(); ++i) {
    for (size_t j = i + 1; j < ids.size(); ++j) {
      const auto a = _markers.find(ids[i]);
      const auto b = _markers.find(ids[j]);
      if (a == _markers.end() || b == _markers.end()) continue;
      const double expected = std::hypot(a->second.x - b->second.x,
                                         a->second.y - b->second.y);
      const double measured = std::hypot(observed[i][0] - observed[j][0],
                                         observed[i][1] - observed[j][1]);
      if (std::fabs(measured - expected) > _spacing_tolerance_m) {
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
  if (observed.size() < 2 || observed.size() != surveyed.size()) return false;

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
    sum_sq += std::pow(px - surveyed[i][0], 2) + std::pow(py - surveyed[i][1], 2);
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
          if (d.has_pose && _markers.count(d.id)) ++known;
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
        "back of the starting zone - closer than ~0.8 m the markers fall out of "
        "the camera's view entirely.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Detection poses are in the camera optical frame; move them into odom so the
  // fit is against a frame that does not move with the robot.
  geometry_msgs::msg::TransformStamped odom_cam;
  try {
    odom_cam = _tf_buffer->lookupTransform(_odom_frame,
                                           detections->header.frame_id,
                                           detections->header.stamp,
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
    if (!d.has_pose) continue;
    const auto known = _markers.find(d.id);
    if (known == _markers.end()) continue;  // whitelist: ignore stray ids

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
        _odom_frame, _base_frame, tf2::TimePointZero, tf2::durationFromSec(0.3));
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
    response->message = std::string("fix accepted but ") + _odom_frame + " -> " +
                        _base_frame + " is unavailable: " + e.what();
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

} // namespace arena_server
