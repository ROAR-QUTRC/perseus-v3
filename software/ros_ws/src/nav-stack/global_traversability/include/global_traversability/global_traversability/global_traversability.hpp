#pragma once

/// @file global_traversability.hpp
/// @brief Builds a persistent, self-clearing terrain costmap of everywhere the
///        rover has looked, from the raw lidar scan.

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <global_traversability/global_traversability/crater_detection.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <local_traversability/local_traversability/terrain_analysis.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

namespace global_traversability
{
    /// @brief ROS 2 node that classifies short windows of raw MID-360 returns by
    ///        height difference and remembers each cell's verdict, publishing the
    ///        result as one map-wide nav2-consumable nav_msgs/OccupancyGrid.
    ///
    /// THIS USED TO READ THE ACCUMULATED LIO MAP, AND THAT WAS THE PROBLEM. LIO
    /// z-wander writes the same floor into /Laser_map several times, 5-10 cm apart, so
    /// no local minimum there can be trusted as "the ground" -- which is what the old
    /// RANSAC ground fit, ground_margin_m 0.15 and the clearance ablation in
    /// navigation.yaml were all fighting. The map was also append-only: an obstacle
    /// that left never left the costmap.
    ///
    /// CLASSIFY SHORT, REMEMBER LONG. Every update runs local_traversability's
    /// height-difference classifier over only the last point_buffer_s of scans, which
    /// were all measured within drift-free reach of each other, so the lowest return
    /// near a cell really is the floor near it. What is REMEMBERED is the verdict per
    /// cell (log-odds of "obstacle"), never the raw height: a height grid stitched
    /// across minutes would put a cell seen at t = 10 s next to one seen at t = 300 s
    /// and read the drift between them as a step, rebuilding the layered-floor
    /// problem cell by cell. Drift between verdicts can only smear an obstacle
    /// sideways on a revisit, and the log-odds washes that out as soon as the cell is
    /// rescanned.
    ///
    /// SELF-CLEARING, NOT FORGETFUL. A cell the current window observes is pulled
    /// towards obstacle or free by that observation; a cell it does not observe keeps
    /// its last value indefinitely. So a rock that is removed clears the next time
    /// the lidar looks at that spot, and one the rover drove away from stays on the
    /// map for the planner no matter how long ago it was seen.
    ///
    /// DEAF THROUGH A SHOCK. "Drift-free within the window" stops being true when the
    /// rover is jolted or drops off something: the raw scans are motion-smeared, the
    /// LIO pose lags and then settles, and scans a few hundred ms apart see the same
    /// floor at different heights and tilts -- which the height test reads as a step.
    /// local_traversability forgets that in a second; this grid would remember it. So
    /// an IMU spike (tilt rate, or specific force off its running 1 g) empties the
    /// scan buffer and ignores scans until shock_holdoff_s after the last spike. The
    /// map just stops learning for that moment, which costs nothing.
    ///
    /// CRATERS ARE A COST, NOT A WALL. The height-difference test only fires on a drop
    /// the slope allowance cannot explain, so a shallow crater (~0.1 m deep, 0.45 m
    /// across) is invisible to it and a deeper one comes out lethal with a 1 m halo.
    /// crater_detection.hpp finds pits by their walls instead; each verdict is fused
    /// into its own log-odds layer exactly like an obstacle's, and a confirmed crater
    /// is published as a flat crater_cost over its footprint -- no inflation -- so the
    /// planner routes round it when that is cheap and through it when it is not.
    ///
    /// The grid starts at initial_map_length_m around the first robot pose and grows
    /// (grid_map::GridMap::extendToInclude) whenever the scan window reaches past its
    /// edge, so it never needs arena bounds up front.
    class GlobalTraversability : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up I/O.
        /// @param options Node options, supplied by main().
        explicit GlobalTraversability(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default topic the raw lidar scan is read from.
        static inline const std::string DEFAULT_POINTCLOUD_TOPIC = "/livox/lidar";
        /// @brief Default topic the final occupancy costmap is published on.
        static inline const std::string DEFAULT_COSTMAP_TOPIC = "costmap";
        /// @brief Namespace debug layers are published under, one
        /// nav_msgs/OccupancyGrid each.
        static inline const std::string DEFAULT_LAYERS_TOPIC_PREFIX = "layers/";

        /// @brief Default topic the shock detector reads. Empty disables it.
        static inline const std::string DEFAULT_IMU_TOPIC = "/livox/imu/corrected";

        /// @brief Default frame the persistent grid is expressed in.
        /// @details odom, not map: BIEVR-LIO's world frame is odom and nothing
        /// publishes map -> odom (see navigation.yaml).
        static inline const std::string DEFAULT_GLOBAL_FRAME = "odom";
        /// @brief Default frame the scan window is centred on.
        static inline const std::string DEFAULT_ROBOT_FRAME = "base_footprint";

        /// @brief Default cell size of both grids, in metres.
        static constexpr double DEFAULT_RESOLUTION_M = 0.1;
        /// @brief Default side of the persistent grid when it is first allocated, in
        ///        metres. It grows past this on its own.
        static constexpr double DEFAULT_INITIAL_MAP_LENGTH_M = 40.0;
        /// @brief Default period between classifying the scan window and fusing it
        ///        into the persistent grid, in seconds.
        static constexpr double DEFAULT_UPDATE_PERIOD_S = 0.5;
        /// @brief Default period between republishing the costmap, in seconds.
        /// @details Separate from the update period because the publish is the
        /// expensive half once the map is arena-sized: inflation and the OccupancyGrid
        /// conversion both scale with the whole map, the fusion only with the window.
        static constexpr double DEFAULT_PUBLISH_PERIOD_S = 1.0;

        /// @brief Default age at which a buffered scan is dropped, in seconds.
        /// @details How much raw data one classification sees. It must stay short
        /// enough that LIO drift across it is negligible -- that is the entire reason
        /// this node does not read the accumulated map.
        static constexpr double DEFAULT_POINT_BUFFER_S = 1.0;
        /// @brief Default closest return kept, in metres from the sensor.
        static constexpr double DEFAULT_MIN_RANGE_M = 1.0;
        /// @brief Default furthest return kept, in metres from the sensor.
        /// @details Further than local_traversability's, because the planner needs to
        /// see further than the controller; beyond ~10 m a 0.1 m grid is too sparse
        /// for the height test to say much.
        static constexpr double DEFAULT_MAX_RANGE_M = 10.0;
        /// @brief Default height above the sensor beyond which returns are dropped,
        ///        in metres.
        static constexpr double DEFAULT_MAX_SENSOR_HEIGHT_M = 1.5;

        /// @name Classifier thresholds
        /// Same meaning as local_traversability's DEFAULT_* constants of the same
        /// names; see local_traversability.hpp.
        /// @{
        static constexpr double DEFAULT_GROUND_WINDOW_M = 0.5;
        static constexpr double DEFAULT_MAX_SLOPE_DEG = 30.0;
        static constexpr double DEFAULT_MAX_STEP_UP_M = 0.2;
        static constexpr double DEFAULT_MAX_STEP_DOWN_M = 0.15;
        static constexpr double DEFAULT_MAX_HEIGHT_ABOVE_GROUND_M = 0.2;
        static constexpr double DEFAULT_OBSTACLE_HEIGHT_CAP_M = 1.0;
        static constexpr double DEFAULT_GROUND_MARGIN_M = 0.10;
        static constexpr double DEFAULT_MIN_CLEARANCE_M = 0.0;
        /// @brief 2 rather than local's 1: this grid remembers, so a lone return
        ///        that local would forget in a second would stay here until rescanned.
        static constexpr int DEFAULT_MIN_POINTS_PER_CELL = 2;
        static constexpr int DEFAULT_MIN_OBSTACLE_CELLS = 2;
        /// @}

        /// @brief Log-odds added to a cell each update the window calls it an
        ///        obstacle.
        /// @details Above log_odds_occupied on its own, so one sighting is enough to
        /// mark a cell: missing a rock is worse than a spurious one that clears on the
        /// next look.
        static constexpr double DEFAULT_LOG_ODDS_HIT = 0.7;
        /// @brief Log-odds subtracted from a cell each update the window calls it
        ///        free.
        static constexpr double DEFAULT_LOG_ODDS_MISS = 0.4;
        /// @brief Lower clamp on a cell's log-odds.
        /// @details Bounds how long ground that has been seen free many times takes
        /// to register something new placed on it.
        static constexpr double DEFAULT_LOG_ODDS_MIN = -2.0;
        /// @brief Upper clamp on a cell's log-odds.
        /// @details Bounds how long a removed obstacle takes to clear once the lidar
        /// is looking at it: (max - occupied) / miss updates, ~4 s at the defaults.
        static constexpr double DEFAULT_LOG_ODDS_MAX = 3.5;
        /// @brief Log-odds above which a cell is published as an obstacle.
        static constexpr double DEFAULT_LOG_ODDS_OCCUPIED = 0.5;

        /// @name Crater detection
        /// See crater_detection.hpp for what the detector tests; the fusion mirrors the
        /// obstacle log-odds above.
        /// @{
        static constexpr bool DEFAULT_CRATER_ENABLED = true;
        /// @brief tan(12 deg) = 0.21. On the sim terrain, with and without 2 cm of range
        ///        noise, nothing natural reaches it for two windows running, while 0.18
        ///        already does.
        static constexpr double DEFAULT_CRATER_MIN_SLOPE_DEG = 12.0;
        static inline const std::vector<double> DEFAULT_CRATER_RING_EDGES_M = {
            0.30, 0.45, 0.65, 0.90};
        static constexpr int DEFAULT_CRATER_MIN_RING_CELLS = 6;
        static constexpr double DEFAULT_CRATER_MIN_RING_COVER = 0.4;
        static constexpr double DEFAULT_CRATER_GROW_RADIUS_M = 0.1;
        static constexpr double DEFAULT_CRATER_FILL_RADIUS_M = 0.2;
        static constexpr double DEFAULT_CRATER_OBSTACLE_MARGIN_M = 0.2;
        /// @brief Rim cells stand ~0.08 m; a wall is 0.85 m. See
        ///        CraterParameters::demote_max_height_m.
        static constexpr double DEFAULT_CRATER_DEMOTE_MAX_HEIGHT_M = 0.12;
        /// @brief Two hits clear crater_log_odds_occupied; a miss costs much less than a
        ///        hit, because a pit is only seen well from some angles and a cell that
        ///        flags in one window and not the next is still a pit.
        static constexpr double DEFAULT_CRATER_LOG_ODDS_HIT = 0.35;
        static constexpr double DEFAULT_CRATER_LOG_ODDS_MISS = 0.1;
        static constexpr double DEFAULT_CRATER_LOG_ODDS_MIN = -1.0;
        static constexpr double DEFAULT_CRATER_LOG_ODDS_MAX = 1.5;
        static constexpr double DEFAULT_CRATER_LOG_ODDS_OCCUPIED = 0.5;
        /// @brief Cost, 0-99, laid over a confirmed crater.
        static constexpr double DEFAULT_CRATER_COST = 40.0;
        /// @brief Whether a low obstacle cell beside a flagged crater cell is demoted to
        ///        free in the window, so it never reaches the obstacle log-odds.
        static constexpr bool DEFAULT_CRATER_DEMOTES_OBSTACLE = true;
        /// @}

        /// @brief Whether never-observed cells are published as obstacles rather than
        ///        as unknown.
        static constexpr bool DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE = false;

        /// @name Shock detection
        /// @{
        /// @brief Default roll/pitch rate, in the robot frame, above which a shock is
        ///        declared, in rad/s.
        /// @details Yaw is excluded so turning on the spot never trips it. Climbing a
        /// rock at drive speed stays well under 1 rad/s; a jolt spikes far past it.
        static constexpr double DEFAULT_SHOCK_TILT_RATE_RAD_S = 1.0;
        /// @brief Default fractional deviation of |specific force| from its running
        ///        mean (~1 g at rest) above which a shock is declared.
        /// @details A ratio, so it works whether the IMU reports in g or m/s^2. 0.5
        /// trips on < 0.5 g (falling) or > 1.5 g (landing).
        static constexpr double DEFAULT_SHOCK_ACCEL_RATIO = 0.5;
        /// @brief Default time after the last shock sample during which scans are
        ///        ignored, in seconds.
        /// @details At least point_buffer_s, so no window straddles the shock, plus
        /// time for LIO's pose to settle after landing.
        static constexpr double DEFAULT_SHOCK_HOLDOFF_S = 1.5;
        /// @brief Default time before a shock from which scans are also discarded, in
        ///        seconds.
        /// @details A scan is stamped at the start of its ~0.1 s sweep, so one that
        /// began just before the spike is already smeared by it.
        static constexpr double DEFAULT_SHOCK_LEAD_S = 0.2;
        /// @brief Time constant of the running |specific force| reference, in seconds.
        static constexpr double ACCEL_REFERENCE_TAU_S = 2.0;
        /// @brief Time constant of the low-pass on |specific force| before it is
        ///        compared, in seconds. Drops single-sample vibration spikes.
        static constexpr double ACCEL_FILTER_TAU_S = 0.02;
        /// @}

        /// @brief Default radius, around the rover, that the clear_region service
        ///        forgets, in metres.
        static constexpr double DEFAULT_CLEAR_RADIUS_M = 2.5;

        /// @brief Default robot radius used as the inscribed (always-lethal)
        ///        inflation distance, in metres.
        static constexpr double DEFAULT_ROBOT_RADIUS_M = 0.5;
        /// @brief Default distance out to which obstacle cost decays, in metres.
        static constexpr double DEFAULT_INFLATION_RADIUS_M = 0.8;
        /// @brief Default exponential decay rate of inflated cost with distance,
        ///        matching nav2's InflationLayer convention.
        static constexpr double DEFAULT_COST_SCALING_FACTOR = 3.2;

        /// @brief Declares every parameter and copies the values into their matching
        ///        members.
        void _load_parameters();

        /// @brief Transforms, filters and buffers an incoming scan.
        /// @param msg Incoming raw lidar scan, in the sensor's own frame.
        void _pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /// @brief Watches for a shock and, on one, empties the scan buffer and starts
        ///        the hold-off.
        /// @param msg Incoming IMU sample.
        void _imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        /// @brief Whether a scan stamped @p stamp may have been taken during a shock.
        /// @param stamp Scan start time.
        bool _in_shock(const rclcpp::Time& stamp) const;

        /// @brief Drops buffered scans older than _point_buffer_s.
        /// @param now Time the age is measured back from.
        void _expire_clouds(const rclcpp::Time& now);

        /// @brief Classifies the current scan window and fuses the verdicts into the
        ///        persistent grid.
        void _update_map();

        /// @brief Recentres the scan window on the robot and grows the persistent
        ///        grid to cover it, allocating both on the first call.
        /// @param center Robot position in the global frame.
        /// @return False if the geometry could not be established.
        bool _place_window(const grid_map::Position& center);

        /// @brief Adds each observed window cell's verdict to the persistent log-odds.
        void _fuse_window();

        /// @brief Forgets the map within clear_radius_m of the rover.
        /// @details A fallback for false obstacles (dust) the map has remembered: this
        /// grid only un-learns a cell when the lidar looks at it again, so a cloud that
        /// has since settled can wall the rover in until then. Cells go back to "never
        /// observed", so real obstacles inside the radius are re-learnt from the next
        /// scans. The request carries nothing; nav2_msgs/ClearEntireCostmap is used only
        /// because the nav2 behaviour tree can call that type by service name.
        /// @param request Unused.
        /// @param response Unused.
        void _clear_region(
            const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> request,
            std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> response);

        /// @brief Derives obstacle/border from the log-odds, inflates, and publishes
        ///        the costmap and any subscribed debug layers.
        void _publish();

        /// @brief One debug layer's name, occupancy value range and publisher.
        struct LayerPublisher
        {
            std::string layer;
            double min_value;
            double max_value;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
        };

        std::string _pointcloud_topic{DEFAULT_POINTCLOUD_TOPIC};
        std::string _imu_topic{DEFAULT_IMU_TOPIC};
        std::string _global_frame{DEFAULT_GLOBAL_FRAME};
        std::string _robot_frame{DEFAULT_ROBOT_FRAME};

        double _resolution_m{DEFAULT_RESOLUTION_M};
        double _clear_radius_m{DEFAULT_CLEAR_RADIUS_M};
        double _initial_map_length_m{DEFAULT_INITIAL_MAP_LENGTH_M};

        double _point_buffer_s{DEFAULT_POINT_BUFFER_S};
        double _min_range_m{DEFAULT_MIN_RANGE_M};
        double _max_range_m{DEFAULT_MAX_RANGE_M};
        double _max_sensor_height_m{DEFAULT_MAX_SENSOR_HEIGHT_M};

        local_traversability::TerrainParameters _terrain{
            DEFAULT_GROUND_WINDOW_M,
            DEFAULT_MAX_SLOPE_DEG,
            DEFAULT_MAX_STEP_UP_M,
            DEFAULT_MAX_STEP_DOWN_M,
            DEFAULT_MAX_HEIGHT_ABOVE_GROUND_M,
            DEFAULT_OBSTACLE_HEIGHT_CAP_M,
            DEFAULT_GROUND_MARGIN_M,
            DEFAULT_MIN_CLEARANCE_M,
            DEFAULT_MIN_POINTS_PER_CELL,
            DEFAULT_MIN_OBSTACLE_CELLS,
            // Always false for the window: an unobserved window cell is "no
            // evidence", and must not be fused as a hit. treat_unknown_as_obstacle
            // applies to the persistent grid instead, see _treat_unknown_as_obstacle.
            false};
        /// @brief Cells under the rover's own footprint are forced free in the published
        ///        costmap, see local_traversability::SelfClearing.
        local_traversability::SelfClearing _self_clear;
        local_traversability::InflationParameters _inflation{
            DEFAULT_ROBOT_RADIUS_M, DEFAULT_INFLATION_RADIUS_M,
            DEFAULT_COST_SCALING_FACTOR};

        double _log_odds_hit{DEFAULT_LOG_ODDS_HIT};
        double _log_odds_miss{DEFAULT_LOG_ODDS_MISS};
        double _log_odds_min{DEFAULT_LOG_ODDS_MIN};
        double _log_odds_max{DEFAULT_LOG_ODDS_MAX};
        double _log_odds_occupied{DEFAULT_LOG_ODDS_OCCUPIED};
        bool _treat_unknown_as_obstacle{DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE};

        bool _crater_enabled{DEFAULT_CRATER_ENABLED};
        CraterParameters _crater{DEFAULT_CRATER_MIN_SLOPE_DEG,
                                 DEFAULT_CRATER_RING_EDGES_M,
                                 DEFAULT_CRATER_MIN_RING_CELLS,
                                 DEFAULT_CRATER_MIN_RING_COVER,
                                 DEFAULT_CRATER_GROW_RADIUS_M,
                                 DEFAULT_CRATER_FILL_RADIUS_M,
                                 DEFAULT_CRATER_OBSTACLE_MARGIN_M,
                                 DEFAULT_CRATER_DEMOTE_MAX_HEIGHT_M};
        double _crater_log_odds_hit{DEFAULT_CRATER_LOG_ODDS_HIT};
        double _crater_log_odds_miss{DEFAULT_CRATER_LOG_ODDS_MISS};
        double _crater_log_odds_min{DEFAULT_CRATER_LOG_ODDS_MIN};
        double _crater_log_odds_max{DEFAULT_CRATER_LOG_ODDS_MAX};
        double _crater_log_odds_occupied{DEFAULT_CRATER_LOG_ODDS_OCCUPIED};
        double _crater_cost{DEFAULT_CRATER_COST};
        bool _crater_demotes_obstacle{DEFAULT_CRATER_DEMOTES_OBSTACLE};

        double _shock_tilt_rate_rad_s{DEFAULT_SHOCK_TILT_RATE_RAD_S};
        double _shock_accel_ratio{DEFAULT_SHOCK_ACCEL_RATIO};
        double _shock_holdoff_s{DEFAULT_SHOCK_HOLDOFF_S};
        double _shock_lead_s{DEFAULT_SHOCK_LEAD_S};
        /// @brief Rotation from the IMU frame into _robot_frame, looked up once.
        std::optional<Eigen::Matrix3d> _imu_to_robot;
        /// @brief Running mean of |specific force|, i.e. what 1 g reads as.
        std::optional<double> _accel_reference;
        /// @brief Low-passed |specific force|.
        double _accel_filtered{0.0};
        std::optional<rclcpp::Time> _last_imu_stamp;
        /// @brief Stamp of the most recent shock sample, if there has been one.
        std::optional<rclcpp::Time> _last_shock_stamp;

        /// @brief The scan window: rebuilt from scratch on every update, never
        ///        carried over.
        grid_map::GridMap _window;
        /// @brief The persistent grid: log_odds, crater_log_odds and elevation are state,
        ///        everything else is derived from them at publish time.
        grid_map::GridMap _map;
        bool _map_initialised{false};
        std::deque<local_traversability::BufferedCloud> _cloud_buffer;

        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
            _pointcloud_subscription;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmap_publisher;
        std::vector<LayerPublisher> _layer_publishers;
        rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr _clear_service;
        /// @brief One-shot publish scheduled by clear_region.
        rclcpp::TimerBase::SharedPtr _publish_once_timer;
        rclcpp::TimerBase::SharedPtr _update_timer;
        rclcpp::TimerBase::SharedPtr _publish_timer;
    };

}  // namespace global_traversability
