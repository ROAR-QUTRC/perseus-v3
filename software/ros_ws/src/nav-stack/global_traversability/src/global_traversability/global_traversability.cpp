/// @file global_traversability.cpp
/// @brief Implementation of the persistent terrain costmap generator.

#include "global_traversability/global_traversability/global_traversability.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>
#include <vector>

namespace global_traversability
{
    namespace
    {
        constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

        /// @brief QoS depth used for the input scan subscription.
        constexpr int POINTCLOUD_QOS_DEPTH = 5;

        /// @brief How long to wait for the scan's own transform before dropping it.
        constexpr double TRANSFORM_TIMEOUT_S = 0.05;

        /// @brief Seconds between repeats of the TF warnings, which would otherwise
        ///        fire once per scan.
        constexpr int WARN_THROTTLE_MS = 2000;

        /// @brief How far past the scan window the persistent grid grows each time it
        ///        has to, in metres.
        /// @details Growing reallocates and copies the whole grid, so it is done in
        /// generous steps rather than one cell at a time as the rover creeps along.
        constexpr double GROWTH_MARGIN_M = 10.0;

        /// @brief Rounds @p value down to a whole number of @p resolution steps.
        double snap_down(double value, double resolution)
        {
            return std::floor(value / resolution) * resolution;
        }

        /// @brief Rounds @p value up to a whole number of @p resolution steps.
        double snap_up(double value, double resolution)
        {
            return std::ceil(value / resolution) * resolution;
        }

        /// @brief Sets @p map to cover [min, max] exactly, cell-aligned.
        /// @details THE ALIGNMENT RULE BOTH GRIDS OBEY: bounds on whole multiples of
        /// the resolution. grid_map places cell edges at center +/- length/2 + k*res,
        /// so with the bounds snapped every cell edge in either grid lands on the same
        /// lattice, and a window cell maps onto exactly one persistent cell by its
        /// center -- no resampling, no cell straddling two.
        void set_aligned_geometry(grid_map::GridMap& map, const grid_map::Position& min,
                                  const grid_map::Position& max, double resolution)
        {
            const grid_map::Position snapped_min(snap_down(min.x(), resolution),
                                                 snap_down(min.y(), resolution));
            const grid_map::Position snapped_max(snap_up(max.x(), resolution),
                                                 snap_up(max.y(), resolution));
            map.setGeometry(grid_map::Length(snapped_max - snapped_min), resolution,
                            (snapped_min + snapped_max) / 2.0);
        }
    }  // namespace

    GlobalTraversability::GlobalTraversability(const rclcpp::NodeOptions& options)
        : rclcpp::Node("global_traversability", options),
          _window(local_traversability::TERRAIN_LAYERS),
          _map({"log_odds", "elevation", "obstacle", "border", "inflation", "cost"})
    {
        _load_parameters();

        _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        // Best effort, matching what the Livox driver publishes. A reliable
        // subscription against a best-effort publisher is a QoS mismatch, which ROS
        // reports as a subscription that simply never receives anything.
        _pointcloud_subscription =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                _pointcloud_topic,
                rclcpp::QoS(POINTCLOUD_QOS_DEPTH).best_effort(),
                std::bind(&GlobalTraversability::_pointcloud_callback, this,
                          std::placeholders::_1));

        // Transient local so nav2's static layer (or rviz, joining late) picks up the
        // most recent costmap immediately rather than waiting for the next publish.
        const auto latched_qos = rclcpp::QoS(1).transient_local();
        _costmap_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            DEFAULT_COSTMAP_TOPIC, latched_qos);

        // Debug/tuning layers, each its own OccupancyGrid (plain nav_msgs, no
        // grid_map_rviz plugin required) with a fixed value range chosen to make the
        // layer's own units legible.
        const std::vector<LayerPublisher> layer_specs = {
            {"log_odds", _log_odds_min, _log_odds_max, nullptr},
            {"elevation", -1.0, 1.0, nullptr},
            {"border", 0.0, 1.0, nullptr},
            {"obstacle", 0.0, 1.0, nullptr},
            {"inflation", 0.0, 99.0, nullptr},
        };
        for (const auto& spec : layer_specs)
        {
            _layer_publishers.push_back(
                {spec.layer, spec.min_value, spec.max_value,
                 this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                     DEFAULT_LAYERS_TOPIC_PREFIX + spec.layer, latched_qos)});
        }

        // ROS timers on the node clock, NOT wall timers, for the same reason as
        // local_traversability's: scan age is judged against this->now(), which under
        // use_sim_time stops with /clock. A wall timer would keep fusing the same
        // frozen buffer into the map forever, driving every cell it covers to the
        // clamp on one second of data.
        const auto to_duration = [](double seconds)
        {
            return rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(seconds)));
        };
        _update_timer = rclcpp::create_timer(
            this, this->get_clock(),
            to_duration(this->get_parameter("update_period_s").as_double()),
            std::bind(&GlobalTraversability::_update_map, this));
        _publish_timer = rclcpp::create_timer(
            this, this->get_clock(),
            to_duration(this->get_parameter("publish_period_s").as_double()),
            std::bind(&GlobalTraversability::_publish, this));
    }

    void GlobalTraversability::_load_parameters()
    {
        _pointcloud_topic =
            this->declare_parameter("pointcloud_topic", DEFAULT_POINTCLOUD_TOPIC);
        _global_frame = this->declare_parameter("global_frame", DEFAULT_GLOBAL_FRAME);
        _robot_frame = this->declare_parameter("robot_frame", DEFAULT_ROBOT_FRAME);

        _resolution_m = this->declare_parameter("resolution_m", DEFAULT_RESOLUTION_M);
        _initial_map_length_m = this->declare_parameter("initial_map_length_m",
                                                        DEFAULT_INITIAL_MAP_LENGTH_M);

        _point_buffer_s =
            this->declare_parameter("point_buffer_s", DEFAULT_POINT_BUFFER_S);
        _min_range_m = this->declare_parameter("min_range_m", DEFAULT_MIN_RANGE_M);
        _max_range_m = this->declare_parameter("max_range_m", DEFAULT_MAX_RANGE_M);
        _max_sensor_height_m = this->declare_parameter("max_sensor_height_m",
                                                       DEFAULT_MAX_SENSOR_HEIGHT_M);

        _terrain.ground_window_m =
            this->declare_parameter("ground_window_m", DEFAULT_GROUND_WINDOW_M);
        _terrain.max_slope_deg =
            this->declare_parameter("max_slope_deg", DEFAULT_MAX_SLOPE_DEG);
        _terrain.max_step_up_m =
            this->declare_parameter("max_step_up_m", DEFAULT_MAX_STEP_UP_M);
        _terrain.max_step_down_m =
            this->declare_parameter("max_step_down_m", DEFAULT_MAX_STEP_DOWN_M);
        _terrain.max_height_above_ground_m = this->declare_parameter(
            "max_height_above_ground_m", DEFAULT_MAX_HEIGHT_ABOVE_GROUND_M);
        _terrain.obstacle_height_cap_m = this->declare_parameter(
            "obstacle_height_cap_m", DEFAULT_OBSTACLE_HEIGHT_CAP_M);
        _terrain.ground_margin_m =
            this->declare_parameter("ground_margin_m", DEFAULT_GROUND_MARGIN_M);
        _terrain.min_clearance_m =
            this->declare_parameter("min_clearance_m", DEFAULT_MIN_CLEARANCE_M);
        _terrain.min_points_per_cell = this->declare_parameter(
            "min_points_per_cell", DEFAULT_MIN_POINTS_PER_CELL);
        _terrain.min_obstacle_cells =
            this->declare_parameter("min_obstacle_cells", DEFAULT_MIN_OBSTACLE_CELLS);

        _log_odds_hit = this->declare_parameter("log_odds_hit", DEFAULT_LOG_ODDS_HIT);
        _log_odds_miss =
            this->declare_parameter("log_odds_miss", DEFAULT_LOG_ODDS_MISS);
        _log_odds_min = this->declare_parameter("log_odds_min", DEFAULT_LOG_ODDS_MIN);
        _log_odds_max = this->declare_parameter("log_odds_max", DEFAULT_LOG_ODDS_MAX);
        _log_odds_occupied =
            this->declare_parameter("log_odds_occupied", DEFAULT_LOG_ODDS_OCCUPIED);
        _treat_unknown_as_obstacle = this->declare_parameter(
            "treat_unknown_as_obstacle", DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE);

        _inflation.robot_radius_m =
            this->declare_parameter("robot_radius_m", DEFAULT_ROBOT_RADIUS_M);
        _inflation.inflation_radius_m =
            this->declare_parameter("inflation_radius_m", DEFAULT_INFLATION_RADIUS_M);
        _inflation.cost_scaling_factor = this->declare_parameter(
            "cost_scaling_factor", DEFAULT_COST_SCALING_FACTOR);

        this->declare_parameter("update_period_s", DEFAULT_UPDATE_PERIOD_S);
        this->declare_parameter("publish_period_s", DEFAULT_PUBLISH_PERIOD_S);
    }

    void GlobalTraversability::_pointcloud_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Resolved on arrival: TF for this scan's own stamp exists now and may have
        // aged out of the buffer by the time the scan is used.
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = _tf_buffer->lookupTransform(
                _global_frame, msg->header.frame_id, msg->header.stamp,
                rclcpp::Duration::from_seconds(TRANSFORM_TIMEOUT_S));
        }
        catch (const tf2::TransformException& exception)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                 WARN_THROTTLE_MS, "Dropping scan: %s",
                                 exception.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        if (cloud.empty())
        {
            return;
        }

        local_traversability::BufferedCloud buffered;
        buffered.stamp = rclcpp::Time(msg->header.stamp);
        buffered.points = local_traversability::filter_scan(
            cloud, tf2::transformToEigen(transform).cast<float>(),
            {_min_range_m, _max_range_m, _max_sensor_height_m});
        if (buffered.points.empty())
        {
            return;
        }

        _cloud_buffer.push_back(std::move(buffered));
        _expire_clouds(rclcpp::Time(msg->header.stamp));
    }

    void GlobalTraversability::_expire_clouds(const rclcpp::Time& now)
    {
        const rclcpp::Duration max_age =
            rclcpp::Duration::from_seconds(_point_buffer_s);
        while (!_cloud_buffer.empty())
        {
            // Guards a clock jump as much as an old scan: a bag loop or a use_sim_time
            // reset moves `now` backwards, and a scan stamped in the future would
            // otherwise pin the buffer until it caught up.
            const rclcpp::Duration age = now - _cloud_buffer.front().stamp;
            if (age <= max_age && age >= rclcpp::Duration::from_seconds(0.0))
            {
                break;
            }
            _cloud_buffer.pop_front();
        }
    }

    void GlobalTraversability::_update_map()
    {
        // Unlike local_traversability, an empty buffer is not a fault to publish: the
        // persistent grid is still exactly as true as it was, it just stops learning.
        // The publish timer keeps republishing it.
        _expire_clouds(this->now());
        if (_cloud_buffer.empty())
        {
            return;
        }

        geometry_msgs::msg::TransformStamped robot_transform;
        try
        {
            robot_transform = _tf_buffer->lookupTransform(_global_frame, _robot_frame,
                                                          tf2::TimePointZero);
        }
        catch (const tf2::TransformException& exception)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                 WARN_THROTTLE_MS, "Skipping update, no robot pose: %s",
                                 exception.what());
            return;
        }

        if (!_place_window(grid_map::Position(robot_transform.transform.translation.x,
                                              robot_transform.transform.translation.y)))
        {
            return;
        }

        local_traversability::classify_terrain(_window, _cloud_buffer, _terrain);
        _fuse_window();
    }

    bool GlobalTraversability::_place_window(const grid_map::Position& center)
    {
        // Checked because grid_map::GridMap::setGeometry asserts internally on a
        // non-positive length or resolution -- a typo in navigation.yaml would abort
        // the process rather than log anything.
        if (!(_resolution_m > 0.0) || !(_max_range_m > _resolution_m) ||
            !(_initial_map_length_m > _resolution_m))
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),
                                  WARN_THROTTLE_MS,
                                  "Invalid map geometry: resolution_m %.3f, max_range_m "
                                  "%.2f, initial_map_length_m %.2f",
                                  _resolution_m, _max_range_m, _initial_map_length_m);
            return false;
        }

        // Every return is within max_range_m of the sensor, and the ground window
        // reaches ground_window_m past that, so this is the smallest window that
        // loses nothing. Rebuilt around the robot each update and never moved: the
        // window holds no state, see _window.
        const double half_window = _max_range_m + _terrain.ground_window_m;
        const grid_map::Position window_min = (center.array() - half_window).matrix();
        const grid_map::Position window_max = (center.array() + half_window).matrix();
        set_aligned_geometry(_window, window_min, window_max, _resolution_m);
        _window.setFrameId(_global_frame);

        if (!_map_initialised)
        {
            const double half_map = std::max(_initial_map_length_m / 2.0, half_window);
            set_aligned_geometry(_map, (center.array() - half_map).matrix(),
                                 (center.array() + half_map).matrix(), _resolution_m);
            _map.setFrameId(_global_frame);
            // setGeometry leaves every layer NaN, which is exactly "never observed".
            _map_initialised = true;
            return true;
        }

        const grid_map::Position map_min =
            (_map.getPosition().array() - _map.getLength().array() / 2.0).matrix();
        const grid_map::Position map_max =
            (_map.getPosition().array() + _map.getLength().array() / 2.0).matrix();
        if ((window_min.array() >= map_min.array()).all() &&
            (window_max.array() <= map_max.array()).all())
        {
            return true;
        }

        // Grow by hand rather than with GridMap::extendToInclude, whose choice of new
        // center is not guaranteed to keep the lattice set_aligned_geometry relies on.
        // Only the sides the window actually crossed are extended.
        grid_map::Position grown_min = map_min;
        grid_map::Position grown_max = map_max;
        for (int axis = 0; axis < 2; ++axis)
        {
            if (window_min(axis) < map_min(axis))
            {
                grown_min(axis) = window_min(axis) - GROWTH_MARGIN_M;
            }
            if (window_max(axis) > map_max(axis))
            {
                grown_max(axis) = window_max(axis) + GROWTH_MARGIN_M;
            }
        }

        grid_map::GridMap grown(_map.getLayers());
        set_aligned_geometry(grown, grown_min, grown_max, _resolution_m);
        grown.setFrameId(_global_frame);
        for (grid_map::GridMapIterator it(_map); !it.isPastEnd(); ++it)
        {
            grid_map::Position position;
            _map.getPosition(*it, position);
            grid_map::Index grown_index;
            if (!grown.getIndex(position, grown_index))
            {
                continue;
            }
            for (const auto& layer : _map.getLayers())
            {
                grown.at(layer, grown_index) = _map.at(layer, *it);
            }
        }
        _map = std::move(grown);

        RCLCPP_INFO(this->get_logger(), "Grew terrain map to %.1f x %.1f m",
                    _map.getLength().x(), _map.getLength().y());
        return true;
    }

    void GlobalTraversability::_fuse_window()
    {
        const Eigen::MatrixXf& window_border = _window["border"];
        const Eigen::MatrixXf& window_obstacle = _window["obstacle"];
        const Eigen::MatrixXf& window_elevation = _window["elevation"];
        Eigen::MatrixXf& log_odds = _map["log_odds"];
        Eigen::MatrixXf& elevation = _map["elevation"];

        const float hit = static_cast<float>(_log_odds_hit);
        const float miss = static_cast<float>(_log_odds_miss);
        const float lowest = static_cast<float>(_log_odds_min);
        const float highest = static_cast<float>(_log_odds_max);

        for (grid_map::GridMapIterator it(_window); !it.isPastEnd(); ++it)
        {
            const grid_map::Index window_index(*it);
            // Border is "too few returns this window" -- no evidence either way, so
            // the persistent cell keeps whatever it last knew. This one check is what
            // makes the map remember what the lidar is not looking at.
            if (window_border(window_index(0), window_index(1)) > 0.5f)
            {
                continue;
            }

            grid_map::Position position;
            _window.getPosition(window_index, position);
            grid_map::Index index;
            if (!_map.getIndex(position, index))
            {
                continue;
            }

            float& cell = log_odds(index(0), index(1));
            const float prior = std::isnan(cell) ? 0.0f : cell;
            const bool is_obstacle =
                window_obstacle(window_index(0), window_index(1)) > 0.5f;
            cell = std::clamp(prior + (is_obstacle ? hit : -miss), lowest, highest);

            // Latest floor, for the debug layer only. Nothing downstream compares it
            // against its neighbours -- see the class comment for why it must not.
            elevation(index(0), index(1)) =
                window_elevation(window_index(0), window_index(1));
        }
    }

    void GlobalTraversability::_publish()
    {
        if (!_map_initialised)
        {
            return;
        }

        const Eigen::MatrixXf& log_odds = _map["log_odds"];
        _map["border"] = log_odds.array().isNaN().cast<float>().matrix();
        _map["obstacle"] =
            (log_odds.array() > static_cast<float>(_log_odds_occupied))
                .cast<float>()
                .matrix();
        if (_treat_unknown_as_obstacle)
        {
            _map["obstacle"] = _map["obstacle"].cwiseMax(_map["border"]);
        }

        local_traversability::compute_inflation(_map, _inflation);
        local_traversability::compute_final_cost(_map, _treat_unknown_as_obstacle);

        const rclcpp::Time stamp = this->now();

        nav_msgs::msg::OccupancyGrid occupancy_grid;
        local_traversability::to_occupancy_grid(_map, "cost", 0.0, 100.0,
                                                occupancy_grid);
        occupancy_grid.header.stamp = stamp;
        _costmap_publisher->publish(occupancy_grid);

        for (const auto& layer_publisher : _layer_publishers)
        {
            // Converting an arena-sized grid is real work on the Orange Pi; skip the
            // layers nobody is looking at.
            if (layer_publisher.publisher->get_subscription_count() == 0)
            {
                continue;
            }
            nav_msgs::msg::OccupancyGrid layer_grid;
            local_traversability::to_occupancy_grid(
                _map, layer_publisher.layer, layer_publisher.min_value,
                layer_publisher.max_value, layer_grid);
            layer_grid.header.stamp = stamp;
            layer_publisher.publisher->publish(layer_grid);
        }
    }

}  // namespace global_traversability
