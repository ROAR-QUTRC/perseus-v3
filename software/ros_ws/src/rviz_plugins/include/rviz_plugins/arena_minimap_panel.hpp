#pragma once

/// @file arena_minimap_panel.hpp
/// @brief RViz panel drawing a 2D top-down minimap of the arena.

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QImage>
#include <QLabel>
#include <QTimer>
#include <QWidget>
#include <arena_server/arena_server/arena_layout.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <string>
#include <vector>

namespace rviz_plugins
{

    /// @brief A 2D rigid transform (translation + yaw), map <- odom as
    /// arena_server broadcasts it (arena_server.cpp's _map_odom). /arena/robot_pose
    /// already carries this applied; /layers/obstacle and /plan_smoothed do not -
    /// both are published in the odom frame (see navigation.yaml: "There is no map
    /// frame ... every global_frame here is odom"), so this has to be applied to
    /// them here or they draw offset from the zones/rover by however far map and
    /// odom have diverged.
    struct PlanarTransform
    {
        double x{0.0}, y{0.0}, yaw{0.0};
    };

    /// @brief The 2D canvas. Split from the panel so paintEvent stays
    /// self-contained.
    class ArenaMinimapCanvas : public QWidget
    {
        Q_OBJECT

    public:
        explicit ArenaMinimapCanvas(QWidget* parent = nullptr);

        /// @brief Supplies the geometry to draw. Called once, after the layout loads.
        void set_layout(const arena_server::ArenaLayout& layout);

        /// @brief Updates the rover marker.
        /// @param pose Rover pose in the arena frame.
        /// @param stale True to draw it greyed, the link having gone quiet.
        void set_pose(const geometry_msgs::msg::Pose& pose, bool stale);

        /// @brief Drops the rover marker entirely, before any pose has arrived.
        void clear_pose();

        /// @brief Updates the obstacle layer overlay from a costmap layer grid.
        /// @param odom_to_map Transform from the grid's own (odom) frame into the
        /// arena frame the zones/rover are drawn in.
        void set_obstacles(const nav_msgs::msg::OccupancyGrid& grid,
                           const PlanarTransform& odom_to_map);

        /// @brief Updates the smoothed-path overlay.
        /// @param odom_to_map See set_obstacles().
        void set_path(const nav_msgs::msg::Path& path,
                      const PlanarTransform& odom_to_map);

    protected:
        void paintEvent(QPaintEvent* event) override;

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

        // Obstacle layer, rasterised once per received grid (not per paint) into an
        // ARGB image so paintEvent only has to blit it. Origin/resolution are in
        // the grid's own (odom) frame, and odom_to_map is baked in as a QPainter
        // transform at paint time rather than pre-applied to the pixels, because
        // odom_to_map can carry a rotation (see PlanarTransform) that a plain
        // axis-aligned QRectF target can't represent - drawImage(QRectF) alone
        // would silently discard any yaw between odom and map.
        QImage _obstacle_image;
        bool _have_obstacles{false};
        double _obstacle_origin_x{0.0}, _obstacle_origin_y{0.0};
        double _obstacle_resolution{0.0};
        PlanarTransform _obstacle_odom_to_map;

        // Smoothed path, stored in the arena (map) frame in metres, already
        // transformed out of odom by the same odom_to_map, rather than
        // pre-converted to pixels, so it re-projects correctly if the panel is
        // resized between paints.
        std::vector<QPointF> _path_world;
        bool _have_path{false};
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
    class ArenaMinimapPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit ArenaMinimapPanel(QWidget* parent = nullptr);

        /// @brief Loads the layout and subscribes. First point RViz offers a node.
        void onInitialize() override;

        void save(rviz_common::Config config) const override;
        void load(const rviz_common::Config& config) override;

    private Q_SLOTS:
        /// @brief Repaints from the latest pose, on the GUI thread.
        void _refresh();

    private:
        /// @brief Topic carrying the rover pose in the arena frame.
        static inline const QString DEFAULT_TOPIC = "/arena/robot_pose";
        /// @brief Global obstacle costmap layer, published transient_local by
        /// global_traversability (see autonomy_bringup/rviz/base_station.rviz's
        /// "Global Obstacle Layer" display, which reads the same topic).
        static inline const QString OBSTACLE_TOPIC = "/layers/obstacle";
        /// @brief nav2's smoother_server output, republished internally under this
        /// fixed name (not something this repo publishes itself).
        static inline const QString PATH_TOPIC = "/plan_smoothed";
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

        /// @brief Stores the latest obstacle grid for the next refresh.
        void
        _on_obstacles(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);

        /// @brief Stores the latest smoothed path for the next refresh.
        void _on_path(nav_msgs::msg::Path::ConstSharedPtr message);

        /// @brief Resolves the layout path, defaulting to autonomy_bringup's copy.
        std::string _resolve_layout_path() const;

        /// @brief Looks up odom -> map: lookupTransform("map", "odom"), i.e. the
        /// live transform arena_server broadcasts (its _map_odom), read the
        /// direction set_obstacles()/set_path() need. Returns false, leaving out
        /// untouched, if TF has no fix yet (e.g. before the first /arena/localise
        /// call resolves it) or the lookup otherwise fails.
        bool _lookup_odom_to_map(PlanarTransform& out) const;

        ArenaMinimapCanvas* _canvas{nullptr};
        QLabel* _status{nullptr};
        QTimer* _refresh_timer{nullptr};

        rclcpp::Node::SharedPtr _node;
        // Only ever used for the single map<->odom lookup above - deliberately not
        // used to reconstruct the rover pose itself (that stays a plain
        // PoseStamped subscription; see the class doc for why streaming TF over
        // this link for the pose was rejected). The obstacle grid and path are
        // already far heavier payloads than TF's small periodic transform, so the
        // reliability tradeoff that ruled TF out for the pose doesn't apply here.
        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
            _subscription;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
            _obstacle_subscription;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription;

        std::mutex _message_mutex;
        geometry_msgs::msg::PoseStamped::ConstSharedPtr _latest_pose;
        rclcpp::Time _latest_pose_time;
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr _latest_obstacles;
        nav_msgs::msg::Path::ConstSharedPtr _latest_path;

        // Last message actually handed to the canvas, compared by pointer identity
        // in _refresh() so an unchanged latched grid or a quiet path topic does not
        // rebuild the overlay every 100 ms tick for nothing.
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr _applied_obstacles;
        nav_msgs::msg::Path::ConstSharedPtr _applied_path;

        QString _topic{DEFAULT_TOPIC};
        QString _layout_path{DEFAULT_LAYOUT};
    };

}  // namespace rviz_plugins
