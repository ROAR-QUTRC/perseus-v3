/// @file local_traversability.cpp
/// @brief Implementation of the rolling local terrain costmap generator.

#include "local_traversability/local_traversability/local_traversability.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>
#include <vector>

namespace local_traversability
{
    namespace
    {
        constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

        /// @brief QoS depth used for the input scan subscription.
        constexpr int POINTCLOUD_QOS_DEPTH = 5;

        /// @brief How long to wait for the scan's own transform before dropping it.
        /// @details Short on purpose. A scan this node cannot place is worth less than
        /// the update it would delay, and the next one is 100 ms away.
        constexpr double TRANSFORM_TIMEOUT_S = 0.05;

        /// @brief Seconds between repeats of the TF warnings, which would otherwise
        ///        fire once per scan.
        constexpr int WARN_THROTTLE_MS = 2000;
    }  // namespace

    LocalTraversability::LocalTraversability(const rclcpp::NodeOptions& options)
        : rclcpp::Node("local_traversability", options),
          _map({"elevation", "height_max", "point_count", "ground", "step_up",
                "step_down", "height_above_ground", "clearance", "border", "obstacle",
                "inflation", "cost"})
    {
        _load_parameters();

        _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        // Best effort, matching what the Livox driver publishes: a sensor stream this
        // node is free to drop scans from, not a stream it must not miss. A reliable
        // subscription against a best-effort publisher is a QoS mismatch, which ROS
        // reports as a subscription that simply never receives anything.
        _pointcloud_subscription =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                _pointcloud_topic,
                rclcpp::QoS(POINTCLOUD_QOS_DEPTH).best_effort(),
                std::bind(&LocalTraversability::_pointcloud_callback, this,
                          std::placeholders::_1));

        // Volatile, unlike global_traversability's latched costmap. This grid is only
        // meaningful next to the robot pose it was built around, so handing a stale
        // one to a late joiner would be worse than handing it nothing -- the next is
        // one update_period_s away. nav2's static layer must therefore be configured
        // with map_subscribe_transient_local false against this topic.
        const auto qos = rclcpp::QoS(1);
        _costmap_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            DEFAULT_COSTMAP_TOPIC, qos);

        // Debug/tuning layers, each its own OccupancyGrid (plain nav_msgs, no
        // grid_map_rviz plugin required) with a fixed value range chosen to make the
        // layer's own units legible. step_up and step_down are published as two
        // unsigned layers rather than one signed one for the same reason
        // global_traversability splits ridge: rviz's occupancy colour schemes read 0
        // as uninteresting and 100 as extreme, so a signed layer renders its own
        // midpoint -- flat ground -- indistinguishable from a hole.
        const std::vector<LayerPublisher> layer_specs = {
            {"elevation", -1.0, 1.0, nullptr},
            {"step_up", 0.0, 0.5, nullptr},
            {"step_down", 0.0, 0.5, nullptr},
            {"height_above_ground", 0.0, 1.0, nullptr},
            {"clearance", 0.0, 2.0, nullptr},
            {"border", 0.0, 1.0, nullptr},
            {"obstacle", 0.0, 1.0, nullptr},
            {"inflation", 0.0, 99.0, nullptr},
        };
        for (const auto& spec : layer_specs)
        {
            _layer_publishers.push_back(
                {spec.layer, spec.min_value, spec.max_value,
                 this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                     DEFAULT_LAYERS_TOPIC_PREFIX + spec.layer, qos)});
        }

        const double update_period_s =
            this->get_parameter("update_period_s").as_double();
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(update_period_s));
        // A ROS timer on the node clock, NOT create_wall_timer, and the difference is a
        // safety one. Scan age in _expire_clouds is measured against this->now(); under
        // use_sim_time that is /clock, which stops dead when a sim is paused or a bag
        // replay ends. A wall timer keeps firing through that, so every buffered scan
        // stays permanently "1 second old" and the node republishes one frozen costmap
        // at 10 Hz forever -- which nav2's static layer consumes as live terrain, with
        // nothing anywhere reporting a fault. Observed doing exactly that against a bag
        // whose /clock had stopped: 60 grids in 6 s, all carrying one stamp 17 days old,
        // while /livox/lidar delivered nothing.
        //
        // Driving the cadence from the same clock the staleness is judged on makes that
        // state unreachable: if the clock stops, so does this.
        _update_timer = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration(period),
            std::bind(&LocalTraversability::_update_costmap, this));
    }

    void LocalTraversability::_load_parameters()
    {
        _pointcloud_topic =
            this->declare_parameter("pointcloud_topic", DEFAULT_POINTCLOUD_TOPIC);
        _global_frame = this->declare_parameter("global_frame", DEFAULT_GLOBAL_FRAME);
        _robot_frame = this->declare_parameter("robot_frame", DEFAULT_ROBOT_FRAME);

        _resolution_m = this->declare_parameter("resolution_m", DEFAULT_RESOLUTION_M);
        _map_length_m = this->declare_parameter("map_length_m", DEFAULT_MAP_LENGTH_M);

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

        _terrain.min_points_per_cell = this->declare_parameter("min_points_per_cell",
                                                               DEFAULT_MIN_POINTS_PER_CELL);
        _terrain.min_obstacle_cells =
            this->declare_parameter("min_obstacle_cells", DEFAULT_MIN_OBSTACLE_CELLS);
        _terrain.treat_unknown_as_obstacle = this->declare_parameter(
            "treat_unknown_as_obstacle", DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE);

        _inflation.robot_radius_m =
            this->declare_parameter("robot_radius_m", DEFAULT_ROBOT_RADIUS_M);
        _inflation.inflation_radius_m =
            this->declare_parameter("inflation_radius_m", DEFAULT_INFLATION_RADIUS_M);
        _inflation.cost_scaling_factor = this->declare_parameter("cost_scaling_factor",
                                                                 DEFAULT_COST_SCALING_FACTOR);

        this->declare_parameter("update_period_s", DEFAULT_UPDATE_PERIOD_S);
    }

    void LocalTraversability::_pointcloud_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Resolved here rather than at rebuild time: TF for this scan's own stamp
        // exists now and may have aged out of the buffer by the time the scan is
        // used. See the class comment.
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

        BufferedCloud buffered;
        buffered.stamp = rclcpp::Time(msg->header.stamp);
        buffered.points = filter_scan(
            cloud, tf2::transformToEigen(transform).cast<float>(),
            {_min_range_m, _max_range_m, _max_sensor_height_m});
        if (buffered.points.empty())
        {
            return;
        }

        _cloud_buffer.push_back(std::move(buffered));
        _expire_clouds(rclcpp::Time(msg->header.stamp));
    }

    void LocalTraversability::_expire_clouds(const rclcpp::Time& now)
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

    void LocalTraversability::_update_costmap()
    {
        _expire_clouds(this->now());
        if (_cloud_buffer.empty())
        {
            // The input died while the clock kept running. Publish one all-unknown grid
            // rather than simply falling silent: nav2's static layer HOLDS the last map
            // it received, so going quiet would leave the controller steering on terrain
            // that is no longer being measured, indefinitely. An unknown grid reads as
            // free under track_unknown_space false, which withdraws this node's
            // contribution and drops nav2 back to the global costmap alone -- the same
            // behaviour it had before this node existed, which is the right thing to
            // degrade to.
            if (_map_initialised && !_published_empty)
            {
                RCLCPP_WARN(this->get_logger(),
                            "No scan within point_buffer_s (%.2f s) on %s; clearing the "
                            "local terrain costmap",
                            _point_buffer_s, _pointcloud_topic.c_str());
                _map["cost"].setConstant(NaN);
                _publish_costmap(this->now());
                _published_empty = true;
            }
            return;
        }
        _published_empty = false;

        geometry_msgs::msg::TransformStamped robot_transform;
        try
        {
            robot_transform = _tf_buffer->lookupTransform(_global_frame, _robot_frame,
                                                          tf2::TimePointZero);
        }
        catch (const tf2::TransformException& exception)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                 WARN_THROTTLE_MS,
                                 "Skipping update, no robot pose: %s",
                                 exception.what());
            return;
        }

        if (!_recenter_map(
                grid_map::Position(robot_transform.transform.translation.x,
                                   robot_transform.transform.translation.y)))
        {
            return;
        }

        classify_terrain(_map, _cloud_buffer, _terrain);
        compute_inflation(_map, _inflation);
        compute_final_cost(_map, _terrain.treat_unknown_as_obstacle);

        const rclcpp::Time stamp = _cloud_buffer.back().stamp;
        _publish_costmap(stamp);
        _publish_layers(stamp);
    }

    bool LocalTraversability::_recenter_map(const grid_map::Position& center)
    {
        // Snapped to whole cells. An unsnapped center re-samples the same terrain onto
        // a grid offset by a fraction of a cell on every update, which shows up as
        // obstacle cells flickering between neighbours while the rover creeps forward
        // -- and each flicker is a fresh inflation disc for the controller.
        const grid_map::Position snapped(
            std::round(center.x() / _resolution_m) * _resolution_m,
            std::round(center.y() / _resolution_m) * _resolution_m);

        if (!_map_initialised)
        {
            // Checked here because grid_map::GridMap::setGeometry returns void and
            // asserts internally on a non-positive length or resolution -- i.e. a
            // typo in navigation.yaml would abort the process rather than log
            // anything. One bad parameter set should cost nav2 this node, not the
            // whole navigation stack.
            if (!(_resolution_m > 0.0) || !(_map_length_m > _resolution_m))
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Invalid map geometry: map_length_m %.2f at "
                             "resolution_m %.3f",
                             _map_length_m, _resolution_m);
                return false;
            }
            _map.setGeometry(grid_map::Length(_map_length_m, _map_length_m),
                             _resolution_m, snapped);
            _map.setFrameId(_global_frame);
            _map_initialised = true;
            return true;
        }

        // setPosition rather than move(): move() shifts the circular buffer to keep
        // the overlapping data, which is exactly what this node does not want. Every
        // layer is rebuilt from the point buffer immediately below, so carrying the
        // old values across would only risk a stale cell surviving where the new
        // scans happen not to reach.
        _map.setPosition(snapped);
        return true;
    }

    void LocalTraversability::_publish_costmap(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        to_occupancy_grid(_map, "cost", 0.0, 100.0, occupancy_grid);
        occupancy_grid.header.stamp = stamp;
        _costmap_publisher->publish(occupancy_grid);
    }

    void LocalTraversability::_publish_layers(const rclcpp::Time& stamp)
    {
        for (const auto& layer_publisher : _layer_publishers)
        {
            // Unlike the costmap, these exist only for rviz, and converting nine of
            // them at sensor rate is real work on the Orange Pi. Skip the ones nobody
            // is looking at.
            if (layer_publisher.publisher->get_subscription_count() == 0)
            {
                continue;
            }
            nav_msgs::msg::OccupancyGrid occupancy_grid;
            to_occupancy_grid(_map, layer_publisher.layer, layer_publisher.min_value,
                              layer_publisher.max_value, occupancy_grid);
            occupancy_grid.header.stamp = stamp;
            layer_publisher.publisher->publish(occupancy_grid);
        }
    }

}  // namespace local_traversability
