#pragma once

/// @file arena_minimap_panel.hpp
/// @brief RViz panel drawing a 2D top-down minimap of the arena.

#include <memory>
#include <mutex>
#include <string>

#include <QLabel>
#include <QTimer>
#include <QWidget>

#include <arena_server/arena_server/arena_layout.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace rviz_plugins {

/// @brief The 2D canvas. Split from the panel so paintEvent stays self-contained.
class ArenaMinimapCanvas : public QWidget {
  Q_OBJECT

public:
  explicit ArenaMinimapCanvas(QWidget *parent = nullptr);

  /// @brief Supplies the geometry to draw. Called once, after the layout loads.
  void set_layout(const arena_server::ArenaLayout &layout);

  /// @brief Updates the rover marker.
  /// @param pose Rover pose in the arena frame.
  /// @param stale True to draw it greyed, the link having gone quiet.
  void set_pose(const geometry_msgs::msg::Pose &pose, bool stale);

  /// @brief Drops the rover marker entirely, before any pose has arrived.
  void clear_pose();

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  /// @brief Maps arena metres to widget pixels.
  ///
  /// Aspect ratio is preserved and Y is flipped, so north is up and the arena
  /// is not stretched to the panel's shape - a minimap that distorts the arena
  /// is worse than no minimap, because distances read wrong.
  QPointF _to_pixels(double x, double y) const;

  arena_server::ArenaLayout _layout;
  bool _have_layout{false};

  geometry_msgs::msg::Pose _pose;
  bool _have_pose{false};
  bool _stale{false};

  // Arena bounds, derived from the zones themselves rather than configured.
  // Deliberately not read from an arena extent: guidebook 5.6.3 bars a priori
  // wall dimensions, and the zones give a drawing extent without asserting
  // anything about where the walls are.
  double _min_x{0.0}, _max_x{0.0}, _min_y{0.0}, _max_y{0.0};
};

/// @brief Dockable RViz panel showing the arena and the rover from above.
///
/// Draws from its own copy of arena_layout.json and one PoseStamped topic, so
/// the base station needs no map, costmap or mesh from the robot. That is the
/// whole point: over a WiFi link the arena outline is static geometry both ends
/// already have on disk, and only the pose has to travel.
///
/// A Panel rather than a Display on purpose. A Display renders into the 3D
/// scene, which is what a MarkerArray already does; a minimap wants its own
/// fixed overhead 2D view that does not move with the camera.
class ArenaMinimapPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit ArenaMinimapPanel(QWidget *parent = nullptr);

  /// @brief Loads the layout and subscribes. First point RViz offers a node.
  void onInitialize() override;

  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;

private Q_SLOTS:
  /// @brief Repaints from the latest pose, on the GUI thread.
  void _refresh();

private:
  /// @brief Topic carrying the rover pose in the arena frame.
  static inline const QString DEFAULT_TOPIC = "/arena/robot_pose";
  /// @brief Layout file, resolved from autonomy_bringup's share directory.
  static inline const QString DEFAULT_LAYOUT = "";
  static constexpr int SUBSCRIPTION_QUEUE_DEPTH = 5;
  /// @brief Repaint period. The pose arrives at about 5 Hz; 10 Hz redraw keeps
  /// the staleness indication prompt without costing anything noticeable.
  static constexpr int REFRESH_PERIOD_MS = 100;
  /// @brief Seconds without a pose before the rover is drawn as stale.
  static constexpr double POSE_TIMEOUT_S = 3.0;

  /// @brief Stores the pose for the next refresh. Runs on an executor thread.
  void _on_pose(geometry_msgs::msg::PoseStamped::ConstSharedPtr message);

  /// @brief Resolves the layout path, defaulting to autonomy_bringup's copy.
  std::string _resolve_layout_path() const;

  ArenaMinimapCanvas *_canvas{nullptr};
  QLabel *_status{nullptr};
  QTimer *_refresh_timer{nullptr};

  rclcpp::Node::SharedPtr _node;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscription;

  std::mutex _message_mutex;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr _latest_pose;
  rclcpp::Time _latest_pose_time;

  QString _topic{DEFAULT_TOPIC};
  QString _layout_path{DEFAULT_LAYOUT};
};

} // namespace rviz_plugins
