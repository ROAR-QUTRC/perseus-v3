#pragma once

/// @file local_traversability.hpp
/// @brief Builds a small, fast, robot-centred terrain costmap from the raw Livox
///        scan, as the reactive counterpart to global_traversability's map-wide
///        one.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <grid_map_core/grid_map_core.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

namespace local_traversability
{
    /// @brief ROS 2 node that turns the last second of raw MID-360 returns into a
    ///        rolling local terrain costmap, published as one nav2-consumable
    ///        nav_msgs/OccupancyGrid plus per-layer debug grids.
    ///
    /// global_traversability answers "what does the whole arena look like" off the
    /// accumulated LIO map, on a 5 s timer. That map is the wrong input for anything
    /// the rover has to react to: it is seconds stale by construction, and the
    /// accumulation itself is what puts two copies of the same floor 5-10 cm apart
    /// (see ground_margin_m in navigation.yaml). This node is the other half --
    /// small window, raw scans, republished at sensor rate -- and the two are layered
    /// in nav2's local costmap rather than one replacing the other.
    ///
    /// THE VERTICAL TEST IS A HEIGHT DIFFERENCE, NOT A PLANE FIT. global_traversability
    /// fits a ground plane by RANSAC because its input is the accumulated map, where
    /// the same ground appears at several heights at once and no single local minimum
    /// can be trusted as "the floor". A raw scan has no such layering: every point in
    /// it was measured in one sweep, so the lowest return near a cell IS the ground
    /// near that cell. That makes the far cheaper test the better one here --
    /// per cell, how far does this stand above the lowest nearby ground, after
    /// allowing for the slope that ground is permitted to have (see
    /// _compute_height_difference). It costs one windowed minimum instead of a
    /// per-patch RANSAC, which is what makes a 10 Hz update affordable at all, and it
    /// has no random component, so the same scan always produces the same costmap.
    ///
    /// The cost of the raw scan is sparsity: a single MID-360 sweep leaves most cells
    /// of a 0.1 m grid empty. Hence _cloud_buffer -- the last point_buffer_s of scans,
    /// transformed into the odom frame on arrival and rebuilt into the grid on every
    /// update. Transforming on arrival, rather than at rebuild time, is deliberate:
    /// TF for a scan's own stamp is available when the scan arrives and may have aged
    /// out of the buffer by the time it is used.
    ///
    /// Deliberately depends on grid_map_core only, not grid_map_ros, and finds PCL
    /// scoped to COMPONENTS common: see CMakeLists.txt for both, and
    /// _to_occupancy_grid() for the hand-rolled GridMap -> OccupancyGrid conversion
    /// that replaces grid_map_ros's converter.
    class LocalTraversability : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up I/O.
        /// @param options Node options, supplied by main().
        explicit LocalTraversability(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default topic the raw lidar scan is read from.
        /// @details The Livox driver's own output, xfer_format 0 (see
        /// sensors/launch/livox.launch.py), so a plain PointCloud2 rather than a
        /// CustomMsg. Point it at /livox/lidar/filtered to run behind the dust filter
        /// instead.
        static inline const std::string DEFAULT_POINTCLOUD_TOPIC = "/livox/lidar";
        /// @brief Default topic the final occupancy costmap is published on.
        static inline const std::string DEFAULT_COSTMAP_TOPIC = "local_costmap_terrain";
        /// @brief Namespace debug layers are published under, one
        /// nav_msgs/OccupancyGrid each.
        static inline const std::string DEFAULT_LAYERS_TOPIC_PREFIX = "local_layers/";

        /// @brief Default frame the rolling grid is expressed in.
        /// @details odom, not map: BIEVR-LIO's world frame is odom and nothing
        /// publishes map -> odom (see navigation.yaml). Must match nav2's
        /// local_costmap global_frame or the static layer silently drops every grid.
        static inline const std::string DEFAULT_GLOBAL_FRAME = "odom";
        /// @brief Default frame the rolling grid is centred on.
        static inline const std::string DEFAULT_ROBOT_FRAME = "base_footprint";

        /// @brief Default cell size of the generated grid, in metres.
        static constexpr double DEFAULT_RESOLUTION_M = 0.1;
        /// @brief Default side length of the square rolling window, in metres.
        static constexpr double DEFAULT_MAP_LENGTH_M = 8.0;
        /// @brief Default period between rebuilds of the grid, in seconds.
        static constexpr double DEFAULT_UPDATE_PERIOD_S = 0.1;

        /// @brief Default age at which a buffered scan is dropped, in seconds.
        /// @details The one dial that trades density against reactivity. Longer fills
        /// more cells and rides through occlusion, but keeps something that has moved
        /// out of the way on the map for that long; the grid holds no state of its own
        /// beyond this, so it is also how long anything stale can survive.
        static constexpr double DEFAULT_POINT_BUFFER_S = 1.0;
        /// @brief Default closest return kept, in metres from the sensor.
        ///        Below this the scan is hitting the rover itself.
        static constexpr double DEFAULT_MIN_RANGE_M = 0.5;
        /// @brief Default furthest return kept, in metres from the sensor.
        static constexpr double DEFAULT_MAX_RANGE_M = 6.0;
        /// @brief Default height above the sensor beyond which returns are discarded,
        ///        in metres.
        /// @details Applied in the sensor frame, before the odom transform, so it is
        /// unaffected by LIO z-drift. Drops ceiling/canopy returns that would
        /// otherwise dominate a cell's maximum height.
        static constexpr double DEFAULT_MAX_SENSOR_HEIGHT_M = 1.5;

        /// @brief Default radius of the window the local ground reference is taken
        ///        over, in metres.
        /// @details Must be comfortably larger than the obstacles being detected or a
        /// boulder becomes its own ground and disappears, and small enough that real
        /// terrain inside it is close to planar. Same trade as
        /// global_traversability's ransac_patch_m, and the dominant cost in the node:
        /// the ground pass is O(cells * (2 * radius / resolution + 1)^2).
        static constexpr double DEFAULT_GROUND_WINDOW_M = 0.5;
        /// @brief Default slope the ground itself is allowed to have, in degrees.
        /// @details Not an obstacle test in its own right -- it is the allowance
        /// subtracted from every height difference, so that ground legitimately
        /// rising at this rate reads as flat. Raising it hides real steps at the
        /// window's edge; lowering it flags open ground on a gradient.
        static constexpr double DEFAULT_MAX_SLOPE_DEG = 30.0;

        /// @brief Default rise of a cell's own floor above the surrounding ground,
        ///        above which it is an obstacle, in metres.
        static constexpr double DEFAULT_MAX_STEP_UP_M = 0.12;
        /// @brief Default drop below the surrounding ground, beyond which a cell is an
        ///        obstacle, in metres. Zero disables the test.
        /// @details Negative obstacles: a hole or a drop-off the rover would fall into.
        /// Weaker evidence than a step up, because the usual signature of a real
        /// drop-off is no returns at all rather than low ones.
        static constexpr double DEFAULT_MAX_STEP_DOWN_M = 0.15;
        /// @brief Default height of structure standing in a cell, above which it is an
        ///        obstacle, in metres.
        /// @details The complement to step_up: a pole or a wall face leaves the cell's
        /// own floor at ground level, so only the cell's highest return shows it.
        static constexpr double DEFAULT_MAX_HEIGHT_ABOVE_GROUND_M = 0.12;
        /// @brief Default height above local ground beyond which a return is treated
        ///        as overhead structure and ignored, in metres.
        /// @details Keeps something the rover drives under out of the height test.
        /// It must stay below the point where a wall would be excluded by it, which
        /// is why min_clearance_m exists as the separate test for "can the rover fit
        /// under this at all".
        static constexpr double DEFAULT_OBSTACLE_HEIGHT_CAP_M = 1.0;

        /// @brief Default height a return must clear local ground by before it counts
        ///        toward overhead clearance, in metres.
        static constexpr double DEFAULT_GROUND_MARGIN_M = 0.10;
        /// @brief Default overhead clearance the robot needs to pass under a cell, in
        ///        metres. Zero disables the test.
        /// @details Much safer here than in global_traversability, where this test
        /// alone accounted for 1079 of the flagged cells because the accumulated map
        /// holds two copies of the floor and the upper one reads as a low ceiling. A
        /// raw scan has one copy, so a low clearance here is a real low ceiling.
        static constexpr double DEFAULT_MIN_CLEARANCE_M = 0.6;

        /// @brief Default minimum returns for a cell to be considered observed rather
        ///        than unknown.
        /// @details 1, against global_traversability's 2. A single sweep over a 0.1 m
        /// grid is sparse enough that demanding a second return mostly erases real,
        /// distant ground.
        static constexpr int DEFAULT_MIN_POINTS_PER_CELL = 1;
        /// @brief Default smallest run of connected obstacle cells that is published;
        ///        anything smaller is erased. 1 disables the filter.
        /// @details Same morphological opening as global_traversability's, and needed
        /// for the opposite reason: not map layering, but the single spurious returns
        /// (dust, rain, a mirror-like surface) that a raw scan has and an accumulated
        /// map has already averaged away.
        static constexpr int DEFAULT_MIN_OBSTACLE_CELLS = 2;
        /// @brief Whether unobserved cells are published as obstacles rather than as
        ///        unknown.
        /// @details false, and it matters more here than globally: most of a local
        /// window is unobserved at any instant (occlusion, beyond range, behind the
        /// rover), and publishing that as lethal would box the rover in every cycle.
        static constexpr bool DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE = false;

        /// @brief Default robot radius used as the inscribed (always-lethal)
        ///        inflation distance, in metres.
        static constexpr double DEFAULT_ROBOT_RADIUS_M = 0.5;
        /// @brief Default distance out to which obstacle cost decays, in metres.
        static constexpr double DEFAULT_INFLATION_RADIUS_M = 0.8;
        /// @brief Default exponential decay rate of inflated cost with distance,
        ///        matching nav2's InflationLayer convention.
        static constexpr double DEFAULT_COST_SCALING_FACTOR = 3.2;

        /// @brief One scan, already transformed into the global frame.
        struct BufferedCloud
        {
            rclcpp::Time stamp;
            std::vector<Eigen::Vector3f> points;
        };

        /// @brief Declares every parameter and copies the values into their matching
        ///        members.
        void _load_parameters();

        /// @brief Transforms, filters and buffers an incoming scan.
        /// @param msg Incoming raw lidar scan, in the sensor's own frame.
        void _pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /// @brief Drops buffered scans older than _point_buffer_s.
        /// @param now Time the age is measured back from.
        void _expire_clouds(const rclcpp::Time& now);

        /// @brief Rebuilds the rolling grid and costmap from the buffered scans.
        void _update_costmap();

        /// @brief Recentres the grid on the robot, allocating it on the first call.
        /// @param center Robot position in the global frame.
        /// @return False if the geometry could not be established.
        bool _recenter_map(const grid_map::Position& center);

        /// @brief Bins every buffered point into elevation/height_max/point_count.
        void _accumulate_elevation();

        /// @brief Fills the ground, step_up and step_down layers by comparing each
        ///        cell's floor against the lowest nearby ground, discounting the
        ///        slope that ground is allowed to have.
        void _compute_height_difference();

        /// @brief Fills height_above_ground and clearance from the buffered points,
        ///        once the ground reference is known.
        void _compute_structure();

        /// @brief Marks cells with too few returns as border/unknown.
        void _compute_border();

        /// @brief Combines step_up, step_down, height_above_ground, clearance and
        ///        border into a binary obstacle layer.
        void _compute_obstacle();

        /// @brief Erases obstacle blobs smaller than _min_obstacle_cells, in place.
        void _prune_small_obstacles();

        /// @brief Runs a two-pass chamfer distance transform off the obstacle layer
        ///        and turns the result into a decaying inflation cost.
        void _compute_inflation();

        /// @brief Folds obstacle, inflation and border into the final 0-100 (or
        ///        NaN/unknown) cost layer that gets exported as the occupancy costmap.
        void _compute_final_cost();

        /// @brief Fills a nav_msgs/OccupancyGrid from one grid_map layer, linearly
        ///        mapping [min_value, max_value] to the occupancy range [0, 100] and
        ///        NaN cells to -1 (unknown). Built directly on grid_map_core's own
        ///        getIndex(), rather than grid_map_ros's converter -- see
        ///        CMakeLists.txt for why.
        /// @param layer Name of the grid_map layer to convert.
        /// @param min_value Layer value mapped to occupancy 0.
        /// @param max_value Layer value mapped to occupancy 100.
        /// @param occupancy_grid_out Receives the converted grid.
        void
        _to_occupancy_grid(const std::string& layer, double min_value,
                           double max_value,
                           nav_msgs::msg::OccupancyGrid& occupancy_grid_out) const;

        /// @brief Publishes the cost layer as the local nav2-consumable costmap.
        /// @param stamp Timestamp to publish the message with.
        void _publish_costmap(const rclcpp::Time& stamp);

        /// @brief Publishes every debug layer as its own OccupancyGrid, for
        ///        inspection/tuning in rviz.
        /// @param stamp Timestamp to publish the messages with.
        void _publish_layers(const rclcpp::Time& stamp);

        /// @brief One debug layer's name, occupancy value range and publisher.
        struct LayerPublisher
        {
            std::string layer;
            double min_value;
            double max_value;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
        };

        std::string _pointcloud_topic{DEFAULT_POINTCLOUD_TOPIC};
        std::string _global_frame{DEFAULT_GLOBAL_FRAME};
        std::string _robot_frame{DEFAULT_ROBOT_FRAME};

        double _resolution_m{DEFAULT_RESOLUTION_M};
        double _map_length_m{DEFAULT_MAP_LENGTH_M};

        double _point_buffer_s{DEFAULT_POINT_BUFFER_S};
        double _min_range_m{DEFAULT_MIN_RANGE_M};
        double _max_range_m{DEFAULT_MAX_RANGE_M};
        double _max_sensor_height_m{DEFAULT_MAX_SENSOR_HEIGHT_M};

        double _ground_window_m{DEFAULT_GROUND_WINDOW_M};
        double _max_slope_deg{DEFAULT_MAX_SLOPE_DEG};

        double _max_step_up_m{DEFAULT_MAX_STEP_UP_M};
        double _max_step_down_m{DEFAULT_MAX_STEP_DOWN_M};
        double _max_height_above_ground_m{DEFAULT_MAX_HEIGHT_ABOVE_GROUND_M};
        double _obstacle_height_cap_m{DEFAULT_OBSTACLE_HEIGHT_CAP_M};

        double _ground_margin_m{DEFAULT_GROUND_MARGIN_M};
        double _min_clearance_m{DEFAULT_MIN_CLEARANCE_M};

        int _min_points_per_cell{DEFAULT_MIN_POINTS_PER_CELL};
        int _min_obstacle_cells{DEFAULT_MIN_OBSTACLE_CELLS};
        bool _treat_unknown_as_obstacle{DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE};

        double _robot_radius_m{DEFAULT_ROBOT_RADIUS_M};
        double _inflation_radius_m{DEFAULT_INFLATION_RADIUS_M};
        double _cost_scaling_factor{DEFAULT_COST_SCALING_FACTOR};

        grid_map::GridMap _map;
        bool _map_initialised{false};
        /// @brief Whether the "input is dead" empty grid has already been published.
        /// @details Latches so the warning and the clearing grid are emitted once per
        /// outage rather than at the update rate.
        bool _published_empty{false};
        std::deque<BufferedCloud> _cloud_buffer;

        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
            _pointcloud_subscription;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmap_publisher;
        std::vector<LayerPublisher> _layer_publishers;
        rclcpp::TimerBase::SharedPtr _update_timer;
    };

}  // namespace local_traversability
