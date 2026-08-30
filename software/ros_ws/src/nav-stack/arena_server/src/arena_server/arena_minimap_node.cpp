/// @file arena_minimap_node.cpp
/// @brief Base-station minimap: draws the arena locally, tracks the robot pose.

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "arena_server/arena_server/arena_layout.hpp"

namespace arena_server {

/// @brief Renders the arena and the rover for the base station.
///
/// This exists so the base station does NOT need the robot's map. It reads the
/// same arena_layout.json the robot reads and draws the arena from that local
/// copy; the only thing that crosses the link is /arena/robot_pose, one small
/// PoseStamped at a few Hz. No costmap, no mesh, no map server.
///
/// It deliberately does not subscribe to TF either. Reconstructing the pose from
/// map -> odom -> base_link would mean streaming both, and would put the base
/// station's display at the mercy of TF timing over a lossy WiFi link. One
/// absolute pose in the arena frame is all a minimap needs.
class ArenaMinimap : public rclcpp::Node {
public:
  ArenaMinimap() : Node("arena_minimap") {
    const auto layout_file = declare_parameter<std::string>("layout_file", "");
    std::string error;
    if (!ArenaLayout::load(layout_file, _layout, error)) {
      RCLCPP_FATAL(get_logger(), "cannot load arena layout: %s", error.c_str());
      throw std::runtime_error("arena layout: " + error);
    }
    RCLCPP_INFO(get_logger(), "layout from %s", layout_file.c_str());
    // Compare this line against arena_server's to confirm both ends agree. If
    // they differ, the base station is drawing an arena the robot is not in.
    RCLCPP_INFO(get_logger(), "layout is %s", _layout.summary().c_str());

    _zone_alpha = declare_parameter<double>("zone_alpha", 0.35);
    _zone_height = declare_parameter<double>("zone_height_m", 0.20);
    _zone_thickness = declare_parameter<double>("zone_thickness_m", 0.04);
    _labels = declare_parameter<bool>("zone_labels", true);
    _robot_radius = declare_parameter<double>("robot_marker_radius_m", 0.35);
    _pose_timeout_s = declare_parameter<double>("pose_timeout_s", 3.0);

    const auto latched = rclcpp::QoS(1).transient_local();
    _zones_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("zones_topic", "/minimap/zones"), latched);
    _robot_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("robot_topic", "/minimap/robot"),
        rclcpp::QoS(5));

    _pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("robot_pose_topic", "/arena/robot_pose"),
        rclcpp::QoS(5),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          _last_pose = msg;
          _last_pose_time = now();
        });

    // The rover marker is drawn on a timer, NOT from the subscription callback.
    // Drawing it in the callback looks equivalent and is not: when the link
    // drops the callback stops firing, so the staleness check below could never
    // run and the marker would sit in RViz at full brightness forever, showing
    // a confident position that stopped updating minutes ago. A timer keeps
    // redrawing whether or not poses arrive, which is the only way "stale" can
    // ever be displayed.
    const double robot_hz = declare_parameter<double>("robot_redraw_hz", 5.0);
    _robot_timer = create_wall_timer(
        std::chrono::duration<double>(1.0 / robot_hz),
        [this]() { _publish_robot(); });

    // Same reasoning as arena_server: the topic is transient local, but RViz's
    // MarkerArray display subscribes VOLATILE and so never sees a message
    // published before it connected. Republishing is what makes the display
    // work without anyone having to change its QoS.
    const double hz = declare_parameter<double>("zone_republish_hz", 1.0);
    _zones_timer = create_wall_timer(std::chrono::duration<double>(1.0 / hz),
                                     [this]() { _publish_zones(); });
    _publish_zones();

    RCLCPP_INFO(get_logger(),
                "arena_minimap up: drawing %zu zones locally, tracking the "
                "rover from a pose topic only",
                _layout.zones.size());
  }

private:
  void _publish_zones() {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    for (const auto &z : _layout.zones) {
      const double hx = z.width * 0.5, hy = z.height * 0.5;
      const double t = _zone_thickness, h = _zone_height;
      const double edges[4][4] = {{z.x, z.y - hy, 2.0 * hx + t, t},
                                  {z.x, z.y + hy, 2.0 * hx + t, t},
                                  {z.x - hx, z.y, t, 2.0 * hy + t},
                                  {z.x + hx, z.y, t, 2.0 * hy + t}};
      for (const auto &e : edges) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = _layout.frame;
        m.header.stamp = now();
        m.ns = "minimap_zones";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = e[0];
        m.pose.position.y = e[1];
        m.pose.position.z = h * 0.5;
        m.pose.orientation.w = 1.0;
        m.scale.x = e[2];
        m.scale.y = e[3];
        m.scale.z = h;
        m.color.r = z.color[0];
        m.color.g = z.color[1];
        m.color.b = z.color[2];
        m.color.a = static_cast<float>(_zone_alpha);
        array.markers.push_back(m);
      }
      if (_labels) {
        visualization_msgs::msg::Marker l;
        l.header.frame_id = _layout.frame;
        l.header.stamp = now();
        l.ns = "minimap_zone_labels";
        l.id = id++;
        l.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        l.action = visualization_msgs::msg::Marker::ADD;
        l.pose.position.x = z.x;
        l.pose.position.y = z.y;
        l.pose.position.z = _zone_height + 0.20;
        l.pose.orientation.w = 1.0;
        l.scale.z = 0.20;
        l.color.r = z.color[0];
        l.color.g = z.color[1];
        l.color.b = z.color[2];
        l.color.a = 1.0f;
        l.text = z.name;
        array.markers.push_back(l);
      }
    }
    _zones_pub->publish(array);
    RCLCPP_INFO_ONCE(get_logger(), "publishing %zu minimap markers",
                     array.markers.size());
  }

  void _publish_robot() {
    if (!_last_pose) return;
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker body;
    body.header.frame_id = _layout.frame;
    body.header.stamp = now();
    body.ns = "minimap_robot";
    body.id = 0;
    body.type = visualization_msgs::msg::Marker::CYLINDER;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose = _last_pose->pose;
    body.pose.position.z = 0.10;
    body.scale.x = body.scale.y = _robot_radius * 2.0;
    body.scale.z = 0.20;
    // Stale poses are drawn grey rather than hidden. On a lossy link "the rover
    // is somewhere it was 10 seconds ago" is useful; a marker that silently
    // vanishes tells the operator nothing about why.
    const bool stale =
        (now() - _last_pose_time).seconds() > _pose_timeout_s;
    body.color.r = stale ? 0.55f : 0.10f;
    body.color.g = stale ? 0.55f : 0.85f;
    body.color.b = stale ? 0.55f : 0.95f;
    body.color.a = 0.9f;
    array.markers.push_back(body);

    // Heading, so the operator can see which way it is pointing - the whole
    // reason a minimap beats a coordinate readout.
    visualization_msgs::msg::Marker heading = body;
    heading.id = 1;
    heading.type = visualization_msgs::msg::Marker::ARROW;
    heading.scale.x = _robot_radius * 2.5;
    heading.scale.y = 0.08;
    heading.scale.z = 0.08;
    heading.pose.position.z = 0.22;
    array.markers.push_back(heading);

    _robot_pub->publish(array);
  }

  ArenaLayout _layout;
  double _zone_alpha{0.35};
  double _zone_height{0.20};
  double _zone_thickness{0.04};
  double _robot_radius{0.35};
  double _pose_timeout_s{3.0};
  bool _labels{true};

  geometry_msgs::msg::PoseStamped::ConstSharedPtr _last_pose;
  rclcpp::Time _last_pose_time;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _zones_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _robot_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_sub;
  rclcpp::TimerBase::SharedPtr _zones_timer;
  rclcpp::TimerBase::SharedPtr _robot_timer;
};

} // namespace arena_server

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arena_server::ArenaMinimap>());
  rclcpp::shutdown();
  return 0;
}
