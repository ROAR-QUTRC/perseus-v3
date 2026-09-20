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

        _ground_window_m =
            this->declare_parameter("ground_window_m", DEFAULT_GROUND_WINDOW_M);
        _max_slope_deg =
            this->declare_parameter("max_slope_deg", DEFAULT_MAX_SLOPE_DEG);

        _max_step_up_m =
            this->declare_parameter("max_step_up_m", DEFAULT_MAX_STEP_UP_M);
        _max_step_down_m =
            this->declare_parameter("max_step_down_m", DEFAULT_MAX_STEP_DOWN_M);
        _max_height_above_ground_m = this->declare_parameter(
            "max_height_above_ground_m", DEFAULT_MAX_HEIGHT_ABOVE_GROUND_M);
        _obstacle_height_cap_m = this->declare_parameter(
            "obstacle_height_cap_m", DEFAULT_OBSTACLE_HEIGHT_CAP_M);

        _ground_margin_m =
            this->declare_parameter("ground_margin_m", DEFAULT_GROUND_MARGIN_M);
        _min_clearance_m =
            this->declare_parameter("min_clearance_m", DEFAULT_MIN_CLEARANCE_M);

        _min_points_per_cell = this->declare_parameter("min_points_per_cell",
                                                       DEFAULT_MIN_POINTS_PER_CELL);
        _min_obstacle_cells =
            this->declare_parameter("min_obstacle_cells", DEFAULT_MIN_OBSTACLE_CELLS);
        _treat_unknown_as_obstacle = this->declare_parameter(
            "treat_unknown_as_obstacle", DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE);

        _robot_radius_m =
            this->declare_parameter("robot_radius_m", DEFAULT_ROBOT_RADIUS_M);
        _inflation_radius_m =
            this->declare_parameter("inflation_radius_m", DEFAULT_INFLATION_RADIUS_M);
        _cost_scaling_factor = this->declare_parameter("cost_scaling_factor",
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

        const Eigen::Isometry3f sensor_to_global =
            tf2::transformToEigen(transform).cast<float>();

        // Every filter below is applied in the SENSOR frame, before the transform:
        // range is only meaningful from the sensor origin, and a height cut taken
        // here is immune to the LIO z-drift the global frame carries.
        const float min_range_squared =
            static_cast<float>(_min_range_m * _min_range_m);
        const float max_range_squared =
            static_cast<float>(_max_range_m * _max_range_m);
        const float max_sensor_height = static_cast<float>(_max_sensor_height_m);

        BufferedCloud buffered;
        buffered.stamp = rclcpp::Time(msg->header.stamp);
        buffered.points.reserve(cloud.points.size());
        for (const auto& point : cloud.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z))
            {
                continue;
            }
            const float range_squared =
                point.x * point.x + point.y * point.y + point.z * point.z;
            if (range_squared < min_range_squared ||
                range_squared > max_range_squared)
            {
                continue;
            }
            if (point.z > max_sensor_height)
            {
                continue;
            }
            buffered.points.push_back(
                sensor_to_global * Eigen::Vector3f(point.x, point.y, point.z));
        }

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

        _map["elevation"].setConstant(NaN);
        _map["height_max"].setConstant(NaN);
        _map["point_count"].setConstant(0.0f);
        _map["ground"].setConstant(NaN);
        _map["step_up"].setConstant(NaN);
        _map["step_down"].setConstant(NaN);
        _map["height_above_ground"].setConstant(NaN);
        _map["clearance"].setConstant(NaN);

        _accumulate_elevation();
        _compute_height_difference();
        _compute_structure();
        _compute_border();
        _compute_obstacle();
        _compute_inflation();
        _compute_final_cost();

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

    void LocalTraversability::_accumulate_elevation()
    {
        Eigen::MatrixXf& elevation = _map["elevation"];
        Eigen::MatrixXf& height_max = _map["height_max"];
        Eigen::MatrixXf& point_count = _map["point_count"];

        for (const auto& buffered : _cloud_buffer)
        {
            for (const auto& point : buffered.points)
            {
                grid_map::Index index;
                if (!_map.getIndex(grid_map::Position(point.x(), point.y()), index))
                {
                    continue;
                }

                float& cell_min = elevation(index(0), index(1));
                cell_min =
                    std::isnan(cell_min) ? point.z() : std::min(cell_min, point.z());

                float& cell_max = height_max(index(0), index(1));
                cell_max =
                    std::isnan(cell_max) ? point.z() : std::max(cell_max, point.z());

                point_count(index(0), index(1)) += 1.0f;
            }
        }
    }

    void LocalTraversability::_compute_height_difference()
    {
        const Eigen::MatrixXf& elevation = _map["elevation"];
        Eigen::MatrixXf& ground = _map["ground"];
        Eigen::MatrixXf& step_up = _map["step_up"];
        Eigen::MatrixXf& step_down = _map["step_down"];

        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);
        const double resolution = _map.getResolution();
        const int window_cells =
            std::max(1, static_cast<int>(std::round(_ground_window_m / resolution)));
        const float slope_allowance_per_cell = static_cast<float>(
            std::tan(_max_slope_deg * M_PI / 180.0) * resolution);

        // Precomputed once instead of a sqrt per neighbour per cell: the window is the
        // same for every cell, so the whole (2n+1)^2 table of allowances is too. This
        // is the node's hot loop -- 121 neighbours per cell at the defaults -- and the
        // sqrt was most of it.
        const int window_side = 2 * window_cells + 1;
        std::vector<float> allowance(static_cast<std::size_t>(window_side) *
                                     static_cast<std::size_t>(window_side));
        for (int d_row = -window_cells; d_row <= window_cells; ++d_row)
        {
            for (int d_col = -window_cells; d_col <= window_cells; ++d_col)
            {
                const float distance_cells = std::sqrt(
                    static_cast<float>(d_row * d_row + d_col * d_col));
                allowance[static_cast<std::size_t>(d_row + window_cells) *
                              static_cast<std::size_t>(window_side) +
                          static_cast<std::size_t>(d_col + window_cells)] =
                    distance_cells * slope_allowance_per_cell;
            }
        }

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                const float center = elevation(row, col);
                if (std::isnan(center))
                {
                    continue;
                }

                // THE HEIGHT-DIFFERENCE TEST. If the ground may slope at up to
                // max_slope_deg, then a neighbour whose floor sits at elevation(n),
                // d cells away, permits the ground under this cell to be anywhere in
                // elevation(n) +/- d * tan(slope). The tightest of those upper bounds
                // over the whole window is the highest this cell's floor could be and
                // still be ground; anything above it is something standing there.
                // The symmetric lower bound gives the drop-off test.
                //
                // Note the window includes the cell itself (d = 0), so upper_bound is
                // never above center and lower_bound never below it: both steps come
                // out non-negative without a clamp.
                float upper_bound = std::numeric_limits<float>::max();
                float lower_bound = std::numeric_limits<float>::lowest();

                const int row_begin = std::max(0, row - window_cells);
                const int row_end = std::min(rows - 1, row + window_cells);
                const int col_begin = std::max(0, col - window_cells);
                const int col_end = std::min(cols - 1, col + window_cells);

                for (int neighbour_row = row_begin; neighbour_row <= row_end;
                     ++neighbour_row)
                {
                    const std::size_t allowance_row =
                        static_cast<std::size_t>(neighbour_row - row + window_cells) *
                        static_cast<std::size_t>(window_side);
                    for (int neighbour_col = col_begin; neighbour_col <= col_end;
                         ++neighbour_col)
                    {
                        const float neighbour = elevation(neighbour_row,
                                                          neighbour_col);
                        if (std::isnan(neighbour))
                        {
                            continue;
                        }
                        const float slack =
                            allowance[allowance_row +
                                      static_cast<std::size_t>(neighbour_col - col +
                                                               window_cells)];
                        upper_bound = std::min(upper_bound, neighbour + slack);
                        lower_bound = std::max(lower_bound, neighbour - slack);
                    }
                }

                ground(row, col) = upper_bound;
                step_up(row, col) = center - upper_bound;
                step_down(row, col) = lower_bound - center;
            }
        }
    }

    void LocalTraversability::_compute_structure()
    {
        const Eigen::MatrixXf& ground = _map["ground"];
        Eigen::MatrixXf& height_above_ground = _map["height_above_ground"];
        Eigen::MatrixXf& clearance = _map["clearance"];

        const float height_cap = static_cast<float>(_obstacle_height_cap_m);
        const float ground_margin = static_cast<float>(_ground_margin_m);

        // A second pass over the same points, rather than reusing height_max from
        // _accumulate_elevation: height_max is measured against nothing, and the two
        // things wanted here are both measured against the ground reference, which
        // only exists now. The tallest return BELOW the overhead cap is the structure
        // standing in the cell; the lowest return ABOVE the ground margin is the
        // ceiling over it.
        for (const auto& buffered : _cloud_buffer)
        {
            for (const auto& point : buffered.points)
            {
                grid_map::Index index;
                if (!_map.getIndex(grid_map::Position(point.x(), point.y()), index))
                {
                    continue;
                }

                const float cell_ground = ground(index(0), index(1));
                if (std::isnan(cell_ground))
                {
                    continue;
                }

                const float above = point.z() - cell_ground;

                if (above <= height_cap)
                {
                    float& cell_height = height_above_ground(index(0), index(1));
                    cell_height =
                        std::isnan(cell_height) ? above : std::max(cell_height, above);
                }

                if (above > ground_margin)
                {
                    float& cell_clearance = clearance(index(0), index(1));
                    cell_clearance = std::isnan(cell_clearance)
                                         ? above
                                         : std::min(cell_clearance, above);
                }
            }
        }
    }

    void LocalTraversability::_compute_border()
    {
        const Eigen::MatrixXf& point_count = _map["point_count"];
        Eigen::MatrixXf& border = _map["border"];
        border = (point_count.array() < static_cast<float>(_min_points_per_cell))
                     .cast<float>()
                     .matrix();
    }

    void LocalTraversability::_compute_obstacle()
    {
        const Eigen::MatrixXf& step_up = _map["step_up"];
        const Eigen::MatrixXf& step_down = _map["step_down"];
        const Eigen::MatrixXf& height_above_ground = _map["height_above_ground"];
        const Eigen::MatrixXf& clearance = _map["clearance"];
        const Eigen::MatrixXf& border = _map["border"];
        Eigen::MatrixXf& obstacle = _map["obstacle"];

        const float max_step_up = static_cast<float>(_max_step_up_m);
        const float max_step_down = static_cast<float>(_max_step_down_m);
        const float max_height = static_cast<float>(_max_height_above_ground_m);
        const float min_clearance = static_cast<float>(_min_clearance_m);

        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (border(row, col) > 0.5f)
                {
                    obstacle(row, col) = _treat_unknown_as_obstacle ? 1.0f : 0.0f;
                    continue;
                }

                const float step_up_value = step_up(row, col);
                const float step_down_value = step_down(row, col);
                const float height_value = height_above_ground(row, col);
                const float clearance_value = clearance(row, col);

                // Zero disables a test rather than making it fire on every cell, which
                // is what the plain comparison would do for the two thresholds a
                // non-negative quantity can equal.
                const bool is_obstacle =
                    (!std::isnan(step_up_value) && step_up_value > max_step_up) ||
                    (max_step_down > 0.0f && !std::isnan(step_down_value) &&
                     step_down_value > max_step_down) ||
                    (!std::isnan(height_value) && height_value > max_height) ||
                    (min_clearance > 0.0f && !std::isnan(clearance_value) &&
                     clearance_value < min_clearance);

                obstacle(row, col) = is_obstacle ? 1.0f : 0.0f;
            }
        }

        _prune_small_obstacles();
    }

    void LocalTraversability::_prune_small_obstacles()
    {
        if (_min_obstacle_cells <= 1)
        {
            return;
        }

        Eigen::MatrixXf& obstacle = _map["obstacle"];
        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);

        // 8-connected flood fill, same as global_traversability's: a sparsely sampled
        // boulder can leave a cell touching its neighbour only at a corner, and
        // splitting it in two would be the filter working against itself.
        Eigen::MatrixXi visited = Eigen::MatrixXi::Zero(rows, cols);
        std::vector<std::pair<int, int>> component;
        std::vector<std::pair<int, int>> stack;

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (visited(row, col) != 0 || obstacle(row, col) <= 0.5f)
                {
                    continue;
                }

                component.clear();
                stack.assign(1, {row, col});
                visited(row, col) = 1;

                while (!stack.empty())
                {
                    const auto [component_row, component_col] = stack.back();
                    stack.pop_back();
                    component.emplace_back(component_row, component_col);

                    for (int d_row = -1; d_row <= 1; ++d_row)
                    {
                        for (int d_col = -1; d_col <= 1; ++d_col)
                        {
                            const int neighbour_row = component_row + d_row;
                            const int neighbour_col = component_col + d_col;
                            if (neighbour_row < 0 || neighbour_row >= rows ||
                                neighbour_col < 0 || neighbour_col >= cols)
                            {
                                continue;
                            }
                            if (visited(neighbour_row, neighbour_col) != 0 ||
                                obstacle(neighbour_row, neighbour_col) <= 0.5f)
                            {
                                continue;
                            }
                            visited(neighbour_row, neighbour_col) = 1;
                            stack.emplace_back(neighbour_row, neighbour_col);
                        }
                    }
                }

                if (component.size() < static_cast<std::size_t>(_min_obstacle_cells))
                {
                    for (const auto& [component_row, component_col] : component)
                    {
                        obstacle(component_row, component_col) = 0.0f;
                    }
                }
            }
        }
    }

    void LocalTraversability::_compute_inflation()
    {
        const Eigen::MatrixXf& obstacle = _map["obstacle"];
        Eigen::MatrixXf& inflation = _map["inflation"];

        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);
        const float resolution = static_cast<float>(_map.getResolution());
        const float infinity = std::numeric_limits<float>::infinity();
        const float diagonal = static_cast<float>(std::sqrt(2.0));

        // Two-pass chamfer distance transform: an approximate Euclidean
        // distance-to-nearest-obstacle in cell units, cheap enough to rerun at sensor
        // rate without extra deps.
        Eigen::MatrixXf distance = Eigen::MatrixXf::Constant(rows, cols, infinity);
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (obstacle(row, col) > 0.5f)
                {
                    distance(row, col) = 0.0f;
                }
            }
        }

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                float& cell = distance(row, col);
                if (row > 0)
                {
                    cell = std::min(cell, distance(row - 1, col) + 1.0f);
                }
                if (col > 0)
                {
                    cell = std::min(cell, distance(row, col - 1) + 1.0f);
                }
                if (row > 0 && col > 0)
                {
                    cell = std::min(cell, distance(row - 1, col - 1) + diagonal);
                }
                if (row > 0 && col + 1 < cols)
                {
                    cell = std::min(cell, distance(row - 1, col + 1) + diagonal);
                }
            }
        }
        for (int row = rows - 1; row >= 0; --row)
        {
            for (int col = cols - 1; col >= 0; --col)
            {
                float& cell = distance(row, col);
                if (row + 1 < rows)
                {
                    cell = std::min(cell, distance(row + 1, col) + 1.0f);
                }
                if (col + 1 < cols)
                {
                    cell = std::min(cell, distance(row, col + 1) + 1.0f);
                }
                if (row + 1 < rows && col + 1 < cols)
                {
                    cell = std::min(cell, distance(row + 1, col + 1) + diagonal);
                }
                if (row + 1 < rows && col > 0)
                {
                    cell = std::min(cell, distance(row + 1, col - 1) + diagonal);
                }
            }
        }

        // Matches nav2's InflationLayer convention, and global_traversability's: lethal
        // out to the robot's own radius, then an exponential decay out to
        // inflation_radius_m, then clear.
        const float inscribed_radius_cells =
            static_cast<float>(_robot_radius_m) / resolution;
        const float inflation_radius_cells =
            static_cast<float>(_inflation_radius_m) / resolution;
        const float scaling_factor = static_cast<float>(_cost_scaling_factor);
        const float robot_radius = static_cast<float>(_robot_radius_m);

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                const float distance_cells = distance(row, col);
                if (distance_cells <= inscribed_radius_cells)
                {
                    inflation(row, col) = 99.0f;
                }
                else if (distance_cells <= inflation_radius_cells)
                {
                    const float distance_m = distance_cells * resolution;
                    inflation(row, col) =
                        99.0f * std::exp(-scaling_factor * (distance_m - robot_radius));
                }
                else
                {
                    inflation(row, col) = 0.0f;
                }
            }
        }
    }

    void LocalTraversability::_compute_final_cost()
    {
        const Eigen::MatrixXf& obstacle = _map["obstacle"];
        const Eigen::MatrixXf& inflation = _map["inflation"];
        const Eigen::MatrixXf& border = _map["border"];
        Eigen::MatrixXf& cost = _map["cost"];

        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (border(row, col) > 0.5f && !_treat_unknown_as_obstacle)
                {
                    cost(row, col) =
                        NaN;  // -> -1 (unknown) once exported as an OccupancyGrid.
                    continue;
                }
                cost(row, col) = obstacle(row, col) > 0.5f
                                     ? 100.0f
                                     : std::min(99.0f, inflation(row, col));
            }
        }
    }

    void LocalTraversability::_to_occupancy_grid(
        const std::string& layer, double min_value, double max_value,
        nav_msgs::msg::OccupancyGrid& occupancy_grid_out) const
    {
        const double resolution = _map.getResolution();
        const grid_map::Length length = _map.getLength();
        const grid_map::Position center = _map.getPosition();

        const int width =
            std::max(1, static_cast<int>(std::round(length.x() / resolution)));
        const int height =
            std::max(1, static_cast<int>(std::round(length.y() / resolution)));

        occupancy_grid_out.header.frame_id = _map.getFrameId();
        occupancy_grid_out.info.resolution = static_cast<float>(resolution);
        occupancy_grid_out.info.width = static_cast<uint32_t>(width);
        occupancy_grid_out.info.height = static_cast<uint32_t>(height);
        occupancy_grid_out.info.origin.position.x = center.x() - length.x() / 2.0;
        occupancy_grid_out.info.origin.position.y = center.y() - length.y() / 2.0;
        occupancy_grid_out.info.origin.position.z = 0.0;
        occupancy_grid_out.info.origin.orientation.w = 1.0;

        occupancy_grid_out.data.assign(
            static_cast<size_t>(width) * static_cast<size_t>(height), -1);

        const Eigen::MatrixXf& values = _map[layer];
        const double range = max_value - min_value;

        // Iterate OccupancyGrid cells (a convention we fully control) rather than
        // grid_map's own row/column order, and ask grid_map's own getIndex() for the
        // matching cell each time -- that way this never has to know or reimplement
        // grid_map's internal index<->world convention, only trust the same lookup
        // already used everywhere else in this file.
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const double world_x =
                    occupancy_grid_out.info.origin.position.x + (x + 0.5) * resolution;
                const double world_y =
                    occupancy_grid_out.info.origin.position.y + (y + 0.5) * resolution;

                grid_map::Index index;
                if (!_map.getIndex(grid_map::Position(world_x, world_y), index))
                {
                    continue;  // stays -1 (unknown)
                }

                const float value = values(index(0), index(1));
                if (std::isnan(value))
                {
                    continue;  // stays -1 (unknown)
                }

                const double normalised =
                    range > 0.0 ? (static_cast<double>(value) - min_value) / range : 0.0;
                const long occupancy =
                    std::lround(std::clamp(normalised, 0.0, 1.0) * 100.0);
                occupancy_grid_out
                    .data[static_cast<size_t>(y) * static_cast<size_t>(width) +
                          static_cast<size_t>(x)] = static_cast<int8_t>(occupancy);
            }
        }
    }

    void LocalTraversability::_publish_costmap(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        _to_occupancy_grid("cost", 0.0, 100.0, occupancy_grid);
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
            _to_occupancy_grid(layer_publisher.layer, layer_publisher.min_value,
                               layer_publisher.max_value, occupancy_grid);
            occupancy_grid.header.stamp = stamp;
            layer_publisher.publisher->publish(occupancy_grid);
        }
    }

}  // namespace local_traversability
