#pragma once

/// @file terrain_analysis.hpp
/// @brief The height-difference terrain pipeline, shared by local_traversability and
///        global_traversability.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <deque>
#include <grid_map_core/grid_map_core.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

namespace local_traversability
{
    /// @brief One scan, already filtered and transformed into the global frame.
    struct BufferedCloud
    {
        rclcpp::Time stamp;
        std::vector<Eigen::Vector3f> points;
    };

    /// @brief Sensor-frame gates applied to a raw scan before it is transformed.
    struct ScanFilter
    {
        /// @brief Closest return kept, in metres from the sensor.
        double min_range_m;
        /// @brief Furthest return kept, in metres from the sensor.
        double max_range_m;
        /// @brief Height above the sensor beyond which returns are dropped, in metres.
        double max_sensor_height_m;
    };

    /// @brief Thresholds of the height-difference classifier. See
    ///        local_traversability.hpp's DEFAULT_* constants for what each one means.
    struct TerrainParameters
    {
        double ground_window_m;
        double max_slope_deg;
        double max_step_up_m;
        double max_step_down_m;
        double max_height_above_ground_m;
        double obstacle_height_cap_m;
        double ground_margin_m;
        double min_clearance_m;
        int min_points_per_cell;
        int min_obstacle_cells;
        bool treat_unknown_as_obstacle;
    };

    /// @brief Inflation shape, same semantics as nav2's InflationLayer.
    struct InflationParameters
    {
        double robot_radius_m;
        double inflation_radius_m;
        double cost_scaling_factor;
    };

    /// @brief Layers classify_terrain() reads and writes; a grid passed to it must
    ///        carry all of them.
    inline const std::vector<std::string> TERRAIN_LAYERS = {
        "elevation", "height_max", "point_count", "ground",
        "step_up", "step_down", "height_above_ground", "clearance",
        "border", "obstacle"};

    /// @brief Applies the sensor-frame gates to a raw scan and transforms the
    ///        survivors into the global frame.
    /// @details Every gate is applied in the SENSOR frame, before the transform:
    /// range is only meaningful from the sensor origin, and a height cut taken there
    /// is immune to the LIO z-drift the global frame carries.
    /// @param cloud Raw scan, in the sensor's own frame.
    /// @param sensor_to_global Pose of the sensor in the global frame at scan time.
    /// @param filter Gates to apply.
    /// @return The kept points, in the global frame.
    std::vector<Eigen::Vector3f> filter_scan(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                             const Eigen::Isometry3f& sensor_to_global,
                                             const ScanFilter& filter);

    /// @brief Rebuilds every TERRAIN_LAYERS layer of @p map from @p clouds, ending in
    ///        a pruned binary obstacle layer plus a border (too few returns) layer.
    /// @details THE VERTICAL TEST IS A HEIGHT DIFFERENCE, NOT A PLANE FIT. Only valid
    /// on points that were measured close together in time -- the lowest return near
    /// a cell is only "the ground" if every return near it saw the same floor. Feed it
    /// a short scan buffer, never an accumulated map.
    /// @param map Grid to fill; its geometry is left untouched.
    /// @param clouds Points to classify, in @p map's frame.
    /// @param params Classifier thresholds.
    void classify_terrain(grid_map::GridMap& map, const std::deque<BufferedCloud>& clouds,
                          const TerrainParameters& params);

    /// @brief Runs a two-pass chamfer distance transform off the "obstacle" layer and
    ///        writes the decaying cost into the "inflation" layer.
    /// @param map Grid carrying both layers.
    /// @param params Inflation shape.
    void compute_inflation(grid_map::GridMap& map, const InflationParameters& params);

    /// @brief Folds "obstacle", "inflation" and "border" into the 0-100 (or
    ///        NaN/unknown) "cost" layer.
    /// @param map Grid carrying all four layers.
    /// @param treat_unknown_as_obstacle Whether border cells cost 100 instead of NaN.
    void compute_final_cost(grid_map::GridMap& map, bool treat_unknown_as_obstacle);

    /// @brief Fills a nav_msgs/OccupancyGrid from one grid_map layer, linearly
    ///        mapping [min_value, max_value] to [0, 100] and NaN cells to -1.
    /// @details Built directly on grid_map_core's own getIndex() rather than
    /// grid_map_ros's converter -- see CMakeLists.txt for why.
    /// @param map Grid to read.
    /// @param layer Name of the layer to convert.
    /// @param min_value Layer value mapped to occupancy 0.
    /// @param max_value Layer value mapped to occupancy 100.
    /// @param occupancy_grid_out Receives the converted grid.
    void to_occupancy_grid(const grid_map::GridMap& map, const std::string& layer,
                           double min_value, double max_value,
                           nav_msgs::msg::OccupancyGrid& occupancy_grid_out);

}  // namespace local_traversability
