/// @file arena_minimap_panel.cpp
/// @brief RViz panel drawing a 2D top-down minimap of the arena.

#include "rviz_plugins/arena_minimap_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_plugins {

namespace {
/// @brief Blank space around the arena, as a fraction of its larger dimension.
constexpr double MARGIN_FRACTION = 0.06;
/// @brief Rover arrow length in metres, drawn to scale with the arena.
constexpr double ARROW_LENGTH_M = 0.9;
constexpr double ARROW_WIDTH_M = 0.55;
} // namespace

ArenaMinimapCanvas::ArenaMinimapCanvas(QWidget *parent) : QWidget(parent) {
  setMinimumSize(220, 220);
  setAutoFillBackground(true);
}

void ArenaMinimapCanvas::set_layout(const arena_server::ArenaLayout &layout) {
  _layout = layout;
  _have_layout = !layout.zones.empty();
  if (!_have_layout)
    return;

  _min_x = _min_y = std::numeric_limits<double>::max();
  _max_x = _max_y = std::numeric_limits<double>::lowest();
  for (const auto &z : _layout.zones) {
    _min_x = std::min(_min_x, z.x - z.width * 0.5);
    _max_x = std::max(_max_x, z.x + z.width * 0.5);
    _min_y = std::min(_min_y, z.y - z.height * 0.5);
    _max_y = std::max(_max_y, z.y + z.height * 0.5);
  }
  const double margin =
      MARGIN_FRACTION * std::max(_max_x - _min_x, _max_y - _min_y);
  _min_x -= margin;
  _max_x += margin;
  _min_y -= margin;
  _max_y += margin;
  update();
}

void ArenaMinimapCanvas::set_pose(const geometry_msgs::msg::Pose &pose,
                                  bool stale) {
  _pose = pose;
  _have_pose = true;
  _stale = stale;
  update();
}

void ArenaMinimapCanvas::clear_pose() {
  _have_pose = false;
  update();
}

QPointF ArenaMinimapCanvas::_to_pixels(double x, double y) const {
  const double span_x = _max_x - _min_x;
  const double span_y = _max_y - _min_y;
  if (span_x <= 0.0 || span_y <= 0.0)
    return {0.0, 0.0};

  // One scale for both axes so the arena keeps its shape; the smaller of the
  // two fits, and the surplus becomes centring offset.
  const double scale = std::min(width() / span_x, height() / span_y);
  const double ox = (width() - span_x * scale) * 0.5;
  const double oy = (height() - span_y * scale) * 0.5;

  // Y is flipped: arena +Y is north, Qt's +y is down the screen.
  return {ox + (x - _min_x) * scale, oy + (_max_y - y) * scale};
}

void ArenaMinimapCanvas::paintEvent(QPaintEvent * /*event*/) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.fillRect(rect(), QColor(28, 28, 32));

  if (!_have_layout) {
    p.setPen(QColor(200, 120, 120));
    p.drawText(rect(), Qt::AlignCenter, "no arena layout loaded");
    return;
  }

  for (const auto &z : _layout.zones) {
    const QPointF tl = _to_pixels(z.x - z.width * 0.5, z.y + z.height * 0.5);
    const QPointF br = _to_pixels(z.x + z.width * 0.5, z.y - z.height * 0.5);
    const QRectF r(tl, br);
    const QColor c(static_cast<int>(z.color[0] * 255),
                   static_cast<int>(z.color[1] * 255),
                   static_cast<int>(z.color[2] * 255));

    // Filled faintly, edges drawn solidly. Zones overlap - the excavation zone
    // contains the starting zone, the construction zone contains the berm - so
    // a heavy fill would bury whichever is drawn first.
    QColor fill = c;
    fill.setAlpha(38);
    p.fillRect(r, fill);

    // Individual edges rather than drawRect, so edges coinciding with an arena
    // wall can be omitted. Drawing every zone closed would trace the arena
    // perimeter, which is a priori wall geometry on screen - see the note on
    // Zone::draw_north. Only zone-to-zone boundaries are informative anyway.
    p.setPen(QPen(c, 1.6));
    if (z.draw_north)
      p.drawLine(r.topLeft(), r.topRight());
    if (z.draw_south)
      p.drawLine(r.bottomLeft(), r.bottomRight());
    if (z.draw_west)
      p.drawLine(r.topLeft(), r.bottomLeft());
    if (z.draw_east)
      p.drawLine(r.topRight(), r.bottomRight());

    if (z.has_edges()) {
      p.setPen(QPen(c.lighter(140), 1.0));
      QFont f = p.font();
      f.setPointSizeF(7.5);
      p.setFont(f);
      p.drawText(r.adjusted(3, 2, -3, -2), Qt::AlignLeft | Qt::AlignTop,
                 QString::fromStdString(z.name));
    }
  }

  if (!_have_pose) {
    p.setPen(QColor(150, 150, 150));
    p.drawText(rect().adjusted(0, 0, -6, -4), Qt::AlignRight | Qt::AlignBottom,
               "waiting for pose");
    return;
  }

  // Rover as an arrow, so heading is visible at a glance - the reason to have a
  // minimap at all rather than a coordinate readout.
  const auto &q = _pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double px = _pose.position.x, py = _pose.position.y;

  // Built in arena metres then mapped, so the arrow scales with the view
  // instead of being a fixed pixel size that misleads when the panel resizes.
  const double l = ARROW_LENGTH_M, w = ARROW_WIDTH_M;
  const QPointF tip = _to_pixels(px + cy * l * 0.5, py + sy * l * 0.5);
  const QPointF left = _to_pixels(px - cy * l * 0.5 - sy * w * 0.5,
                                  py - sy * l * 0.5 + cy * w * 0.5);
  const QPointF right = _to_pixels(px - cy * l * 0.5 + sy * w * 0.5,
                                   py - sy * l * 0.5 - cy * w * 0.5);
  const QPointF notch = _to_pixels(px - cy * l * 0.2, py - sy * l * 0.2);

  QPainterPath arrow;
  arrow.moveTo(tip);
  arrow.lineTo(left);
  arrow.lineTo(notch);
  arrow.lineTo(right);
  arrow.closeSubpath();

  // Grey rather than hidden when the pose has gone quiet: "it was here 10
  // seconds ago" is useful to an operator, whereas a marker that silently
  // vanishes says nothing about why.
  const QColor body = _stale ? QColor(140, 140, 140) : QColor(70, 200, 255);
  p.setBrush(body);
  p.setPen(QPen(body.darker(160), 1.2));
  p.drawPath(arrow);
}

// ─────────────────────────────────────────────────────────────────────────────

ArenaMinimapPanel::ArenaMinimapPanel(QWidget *parent)
    : rviz_common::Panel(parent), _latest_pose_time(0, 0, RCL_ROS_TIME) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(2, 2, 2, 2);
  _canvas = new ArenaMinimapCanvas(this);
  _status = new QLabel("arena minimap", this);
  _status->setStyleSheet("color: #999; font-size: 10px;");
  layout->addWidget(_canvas, 1);
  layout->addWidget(_status, 0);
  setLayout(layout);
}

std::string ArenaMinimapPanel::_resolve_layout_path() const {
  if (!_layout_path.isEmpty())
    return _layout_path.toStdString();
  // Defaults to the copy the robot reads, so out of the box both ends agree.
  try {
    return ament_index_cpp::get_package_share_directory("autonomy_bringup") +
           "/config/arena_layout.json";
  } catch (const std::exception &) {
    return {};
  }
}

void ArenaMinimapPanel::onInitialize() {
  _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  const std::string path = _resolve_layout_path();
  arena_server::ArenaLayout layout;
  std::string error;
  if (!arena_server::ArenaLayout::load(path, layout, error)) {
    // Reported in the panel, not just the log: an operator looking at an empty
    // minimap needs to see why without going to a terminal.
    _status->setText(
        QString("layout failed: %1").arg(QString::fromStdString(error)));
    _status->setStyleSheet("color: #d66; font-size: 10px;");
    RCLCPP_ERROR(_node->get_logger(), "arena minimap: %s", error.c_str());
  } else {
    _canvas->set_layout(layout);
    // Same summary arena_server logs, so a mismatch between the two ends is
    // visible by comparing one line in each log.
    RCLCPP_INFO(_node->get_logger(), "arena minimap layout from %s",
                path.c_str());
    RCLCPP_INFO(_node->get_logger(), "arena minimap layout is %s",
                layout.summary().c_str());
    _status->setText(QString("%1 zones from %2")
                         .arg(layout.zones.size())
                         .arg(QString::fromStdString(path).section('/', -1)));
  }

  _subscription = _node->create_subscription<geometry_msgs::msg::PoseStamped>(
      _topic.toStdString(), SUBSCRIPTION_QUEUE_DEPTH,
      std::bind(&ArenaMinimapPanel::_on_pose, this, std::placeholders::_1));

  _refresh_timer = new QTimer(this);
  connect(_refresh_timer, &QTimer::timeout, this, &ArenaMinimapPanel::_refresh);
  _refresh_timer->start(REFRESH_PERIOD_MS);
}

void ArenaMinimapPanel::_on_pose(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
  // Executor thread: store only. Qt widgets may be touched from the GUI thread
  // alone, so the repaint happens in _refresh().
  std::lock_guard<std::mutex> lock(_message_mutex);
  _latest_pose = message;
  _latest_pose_time = _node->now();
}

void ArenaMinimapPanel::_refresh() {
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(_message_mutex);
    pose = _latest_pose;
    stamp = _latest_pose_time;
  }
  if (!pose) {
    _canvas->clear_pose();
    return;
  }
  // Redrawn on a timer rather than from the callback on purpose: when the link
  // drops the callback stops firing, so a callback-driven repaint could never
  // show staleness - the arrow would sit at full brightness indefinitely,
  // asserting a position that stopped updating minutes ago.
  const double age = (_node->now() - stamp).seconds();
  _canvas->set_pose(pose->pose, age > POSE_TIMEOUT_S);
}

void ArenaMinimapPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", _topic);
  config.mapSetValue("LayoutPath", _layout_path);
}

void ArenaMinimapPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  config.mapGetString("Topic", &_topic);
  config.mapGetString("LayoutPath", &_layout_path);
}

} // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ArenaMinimapPanel, rviz_common::Panel)
