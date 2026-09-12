/// @file arena_minimap_panel.cpp
/// @brief RViz panel drawing a 2D top-down minimap of the arena.

#include "rviz_plugins/arena_minimap_panel.hpp"

#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cstdint>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <utility>

namespace rviz_plugins
{

    namespace
    {
        /// @brief Blank space around the arena, as a fraction of its larger dimension.
        constexpr double MARGIN_FRACTION = 0.06;
        /// @brief Rover arrow length in metres, drawn to scale with the arena.
        constexpr double ARROW_LENGTH_M = 0.9;
        constexpr double ARROW_WIDTH_M = 0.55;
    }  // namespace

    ArenaMinimapCanvas::ArenaMinimapCanvas(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(220, 220);
        setAutoFillBackground(true);
    }

    void ArenaMinimapCanvas::set_layout(const arena_server::ArenaLayout& layout)
    {
        _layout = layout;
        _have_layout = !layout.zones.empty();
        if (!_have_layout)
            return;

        _min_x = _min_y = std::numeric_limits<double>::max();
        _max_x = _max_y = std::numeric_limits<double>::lowest();
        for (const auto& z : _layout.zones)
        {
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

    void ArenaMinimapCanvas::set_pose(const geometry_msgs::msg::Pose& pose,
                                      bool stale)
    {
        _pose = pose;
        _have_pose = true;
        _stale = stale;
        update();
    }

    void ArenaMinimapCanvas::clear_pose()
    {
        _have_pose = false;
        update();
    }

    void ArenaMinimapCanvas::set_obstacles(const nav_msgs::msg::OccupancyGrid& grid,
                                           const PlanarTransform& odom_to_map)
    {
        const int w = static_cast<int>(grid.info.width);
        const int h = static_cast<int>(grid.info.height);
        if (w <= 0 || h <= 0 ||
            grid.data.size() != static_cast<std::size_t>(w) * h)
        {
            _have_obstacles = false;
            update();
            return;
        }

        // Rendered once here rather than per-paint: the grid is transient_local and
        // rarely changes, so paintEvent should only ever have to blit this, not
        // walk every cell 10 times a second. Rows are kept in the grid's own
        // south-to-north order (row 0 = min-y) - unlike a plain axis-aligned draw,
        // the Y-flip and any odom/map yaw are both applied as a single QPainter
        // transform in paintEvent, not baked into the pixels here.
        QImage image(w, h, QImage::Format_ARGB32_Premultiplied);
        image.fill(Qt::transparent);
        for (int row = 0; row < h; ++row)
        {
            auto* pixels = reinterpret_cast<QRgb*>(image.scanLine(row));
            for (int col = 0; col < w; ++col)
            {
                const std::int8_t value = grid.data[row * w + col];
                // Only occupied cells are drawn - free (0) and unknown (-1) space is
                // left transparent so the zone outlines and labels beneath stay
                // legible, rather than burying them under a solid costmap fill.
                if (value <= 0)
                    continue;
                const int alpha = std::clamp(static_cast<int>(value) * 2, 0, 210);
                pixels[col] = qRgba(230, 230, 230, alpha);
            }
        }

        _obstacle_image = std::move(image);
        _obstacle_origin_x = grid.info.origin.position.x;
        _obstacle_origin_y = grid.info.origin.position.y;
        _obstacle_resolution = grid.info.resolution;
        _obstacle_odom_to_map = odom_to_map;
        _have_obstacles = true;
        update();
    }

    void ArenaMinimapCanvas::set_path(const nav_msgs::msg::Path& path,
                                      const PlanarTransform& odom_to_map)
    {
        const double c = std::cos(odom_to_map.yaw), s = std::sin(odom_to_map.yaw);
        _path_world.clear();
        _path_world.reserve(path.poses.size());
        for (const auto& pose : path.poses)
        {
            const double ox = pose.pose.position.x, oy = pose.pose.position.y;
            _path_world.emplace_back(odom_to_map.x + c * ox - s * oy,
                                     odom_to_map.y + s * ox + c * oy);
        }
        _have_path = _path_world.size() >= 2;
        update();
    }

    QPointF ArenaMinimapCanvas::_to_pixels(double x, double y) const
    {
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

    void ArenaMinimapCanvas::paintEvent(QPaintEvent* /*event*/)
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.fillRect(rect(), QColor(28, 28, 32));

        if (!_have_layout)
        {
            p.setPen(QColor(200, 120, 120));
            p.drawText(rect(), Qt::AlignCenter, "no arena layout loaded");
            return;
        }

        for (const auto& z : _layout.zones)
        {
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

            if (z.has_edges())
            {
                p.setPen(QPen(c.lighter(140), 1.0));
                QFont f = p.font();
                f.setPointSizeF(7.5);
                p.setFont(f);
                p.drawText(r.adjusted(3, 2, -3, -2), Qt::AlignLeft | Qt::AlignTop,
                           QString::fromStdString(z.name));
            }
        }

        if (_have_obstacles)
        {
            const double span_x = _max_x - _min_x;
            const double span_y = _max_y - _min_y;
            if (span_x > 0.0 && span_y > 0.0)
            {
                const double scale = std::min(width() / span_x, height() / span_y);
                const double ox = (width() - span_x * scale) * 0.5;
                const double oy = (height() - span_y * scale) * 0.5;

                p.save();
                // Nested coordinate systems, outermost first, mirroring _to_pixels then
                // PlanarTransform then the grid's own pixel spacing. Composed as one
                // QPainter transform (rather than pre-transforming every cell) because
                // the odom -> map leg can carry a yaw, which a plain axis-aligned
                // QRectF target can't represent.
                p.translate(ox - _min_x * scale, oy + _max_y * scale);
                p.scale(scale, -scale);
                p.translate(_obstacle_odom_to_map.x, _obstacle_odom_to_map.y);
                p.rotate(_obstacle_odom_to_map.yaw * 180.0 / M_PI);
                p.translate(_obstacle_origin_x, _obstacle_origin_y);
                p.scale(_obstacle_resolution, _obstacle_resolution);
                p.drawImage(QRectF(0, 0, _obstacle_image.width(),
                                   _obstacle_image.height()),
                            _obstacle_image);
                p.restore();
            }
        }

        if (_have_path)
        {
            QPainterPath path_line;
            path_line.moveTo(_to_pixels(_path_world.front().x(), _path_world.front().y()));
            for (std::size_t i = 1; i < _path_world.size(); ++i)
                path_line.lineTo(_to_pixels(_path_world[i].x(), _path_world[i].y()));
            // Same cyan as the fresh rover arrow below, so the path reads as "this
            // rover's route" rather than an unrelated overlay colour.
            p.setPen(QPen(QColor(70, 200, 255), 2.0));
            p.drawPath(path_line);
        }

        if (!_have_pose)
        {
            p.setPen(QColor(150, 150, 150));
            p.drawText(rect().adjusted(0, 0, -6, -4), Qt::AlignRight | Qt::AlignBottom,
                       "waiting for pose");
            return;
        }

        // Rover as an arrow, so heading is visible at a glance - the reason to have a
        // minimap at all rather than a coordinate readout.
        const auto& q = _pose.orientation;
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

    ArenaMinimapPanel::ArenaMinimapPanel(QWidget* parent)
        : rviz_common::Panel(parent),
          _latest_pose_time(0, 0, RCL_ROS_TIME)
    {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(2, 2, 2, 2);
        _canvas = new ArenaMinimapCanvas(this);
        _status = new QLabel("arena minimap", this);
        _status->setStyleSheet("color: #999; font-size: 10px;");
        layout->addWidget(_canvas, 1);
        layout->addWidget(_status, 0);
        setLayout(layout);
    }

    std::string ArenaMinimapPanel::_resolve_layout_path() const
    {
        if (!_layout_path.isEmpty())
            return _layout_path.toStdString();
        // Defaults to the copy the robot reads, so out of the box both ends agree.
        try
        {
            return ament_index_cpp::get_package_share_directory("autonomy_bringup") +
                   "/config/arena_layout.json";
        }
        catch (const std::exception&)
        {
            return {};
        }
    }

    void ArenaMinimapPanel::onInitialize()
    {
        _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        _tf_buffer = std::make_shared<tf2_ros::Buffer>(_node->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        const std::string path = _resolve_layout_path();
        arena_server::ArenaLayout layout;
        std::string error;
        if (!arena_server::ArenaLayout::load(path, layout, error))
        {
            // Reported in the panel, not just the log: an operator looking at an empty
            // minimap needs to see why without going to a terminal.
            _status->setText(
                QString("layout failed: %1").arg(QString::fromStdString(error)));
            _status->setStyleSheet("color: #d66; font-size: 10px;");
            RCLCPP_ERROR(_node->get_logger(), "arena minimap: %s", error.c_str());
        }
        else
        {
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

        // Transient local, depth 1: matches global_traversability's own publisher
        // QoS (see base_station.rviz's "Global Obstacle Layer" display, fixed for
        // the same reason) so a panel opened after the grid was last published
        // still gets it immediately rather than waiting for the next update cycle.
        _obstacle_subscription =
            _node->create_subscription<nav_msgs::msg::OccupancyGrid>(
                OBSTACLE_TOPIC.toStdString(), rclcpp::QoS(1).transient_local(),
                std::bind(&ArenaMinimapPanel::_on_obstacles, this,
                          std::placeholders::_1));

        _path_subscription = _node->create_subscription<nav_msgs::msg::Path>(
            PATH_TOPIC.toStdString(), SUBSCRIPTION_QUEUE_DEPTH,
            std::bind(&ArenaMinimapPanel::_on_path, this, std::placeholders::_1));

        _refresh_timer = new QTimer(this);
        connect(_refresh_timer, &QTimer::timeout, this, &ArenaMinimapPanel::_refresh);
        _refresh_timer->start(REFRESH_PERIOD_MS);
    }

    void ArenaMinimapPanel::_on_pose(
        geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
    {
        // Executor thread: store only. Qt widgets may be touched from the GUI thread
        // alone, so the repaint happens in _refresh().
        std::lock_guard<std::mutex> lock(_message_mutex);
        _latest_pose = message;
        _latest_pose_time = _node->now();
    }

    void ArenaMinimapPanel::_on_obstacles(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
    {
        std::lock_guard<std::mutex> lock(_message_mutex);
        _latest_obstacles = message;
    }

    void ArenaMinimapPanel::_on_path(nav_msgs::msg::Path::ConstSharedPtr message)
    {
        std::lock_guard<std::mutex> lock(_message_mutex);
        _latest_path = message;
    }

    bool ArenaMinimapPanel::_lookup_odom_to_map(PlanarTransform& out) const
    {
        try
        {
            // TimePointZero: the latest available, not the message's own stamp - the
            // grid/path can be older than the newest TF sample by the time they're
            // applied, and holding out for an exact-time match would just mean
            // occasionally failing this lookup for no benefit on a display that
            // isn't claiming sub-frame synchronisation to begin with.
            const auto tf = _tf_buffer->lookupTransform("map", "odom", tf2::TimePointZero);
            out.x = tf.transform.translation.x;
            out.y = tf.transform.translation.y;
            out.yaw = tf2::getYaw(tf.transform.rotation);
            return true;
        }
        catch (const tf2::TransformException& e)
        {
            RCLCPP_WARN_THROTTLE(_node->get_logger(), *_node->get_clock(), 5000,
                                 "arena minimap: map -> odom not yet available (%s); "
                                 "obstacle layer/path will not update until it is",
                                 e.what());
            return false;
        }
    }

    void ArenaMinimapPanel::_refresh()
    {
        geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
        rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr obstacles;
        nav_msgs::msg::Path::ConstSharedPtr path;
        {
            std::lock_guard<std::mutex> lock(_message_mutex);
            pose = _latest_pose;
            stamp = _latest_pose_time;
            obstacles = _latest_obstacles;
            path = _latest_path;
        }

        if (!pose)
        {
            _canvas->clear_pose();
        }
        else
        {
            // Redrawn on a timer rather than from the callback on purpose: when the
            // link drops the callback stops firing, so a callback-driven repaint
            // could never show staleness - the arrow would sit at full brightness
            // indefinitely, asserting a position that stopped updating minutes ago.
            const double age = (_node->now() - stamp).seconds();
            _canvas->set_pose(pose->pose, age > POSE_TIMEOUT_S);
        }

        // Compared by pointer identity so an unchanged latched grid, or a quiet
        // path topic, does not re-rasterise/re-walk the overlay every 100 ms tick
        // for nothing - only apply what's actually new since the last refresh.
        // Looked up once here, not inside each set_*() call, since both need the
        // exact same transform to stay mutually consistent with each other and
        // with the rover arrow.
        if ((obstacles && obstacles != _applied_obstacles) ||
            (path && path != _applied_path))
        {
            PlanarTransform odom_to_map;
            if (_lookup_odom_to_map(odom_to_map))
            {
                if (obstacles && obstacles != _applied_obstacles)
                {
                    _canvas->set_obstacles(*obstacles, odom_to_map);
                    _applied_obstacles = obstacles;
                }
                if (path && path != _applied_path)
                {
                    _canvas->set_path(*path, odom_to_map);
                    _applied_path = path;
                }
            }
            // On failure, applied_* is deliberately left unset so the same new
            // message is retried on the next tick instead of being silently dropped.
        }
    }

    void ArenaMinimapPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("Topic", _topic);
        config.mapSetValue("LayoutPath", _layout_path);
    }

    void ArenaMinimapPanel::load(const rviz_common::Config& config)
    {
        rviz_common::Panel::load(config);
        config.mapGetString("Topic", &_topic);
        config.mapGetString("LayoutPath", &_layout_path);
    }

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ArenaMinimapPanel, rviz_common::Panel)
