/// @file terrain_analysis.cpp
/// @brief Implementation of the shared height-difference terrain pipeline.

#include "local_traversability/local_traversability/terrain_analysis.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace local_traversability
{
    namespace
    {
        constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

        void accumulate_elevation(grid_map::GridMap& map,
                                  const std::deque<BufferedCloud>& clouds)
        {
            Eigen::MatrixXf& elevation = map["elevation"];
            Eigen::MatrixXf& height_max = map["height_max"];
            Eigen::MatrixXf& point_count = map["point_count"];

            for (const auto& buffered : clouds)
            {
                for (const auto& point : buffered.points)
                {
                    grid_map::Index index;
                    if (!map.getIndex(grid_map::Position(point.x(), point.y()), index))
                    {
                        continue;
                    }

                    float& cell_min = elevation(index(0), index(1));
                    cell_min = std::isnan(cell_min) ? point.z()
                                                    : std::min(cell_min, point.z());

                    float& cell_max = height_max(index(0), index(1));
                    cell_max = std::isnan(cell_max) ? point.z()
                                                    : std::max(cell_max, point.z());

                    point_count(index(0), index(1)) += 1.0f;
                }
            }
        }

        void compute_height_difference(grid_map::GridMap& map,
                                       const TerrainParameters& params)
        {
            const Eigen::MatrixXf& elevation = map["elevation"];
            Eigen::MatrixXf& ground = map["ground"];
            Eigen::MatrixXf& step_up = map["step_up"];
            Eigen::MatrixXf& step_down = map["step_down"];

            const int rows = map.getSize()(0);
            const int cols = map.getSize()(1);
            const double resolution = map.getResolution();
            const int window_cells = std::max(
                1, static_cast<int>(std::round(params.ground_window_m / resolution)));
            const float slope_allowance_per_cell = static_cast<float>(
                std::tan(params.max_slope_deg * M_PI / 180.0) * resolution);

            // Precomputed once instead of a sqrt per neighbour per cell: the window is
            // the same for every cell, so the whole (2n+1)^2 table of allowances is
            // too. This is the hot loop -- 121 neighbours per cell at the defaults --
            // and the sqrt was most of it.
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
                    // max_slope_deg, then a neighbour whose floor sits at
                    // elevation(n), d cells away, permits the ground under this cell to
                    // be anywhere in elevation(n) +/- d * tan(slope). The tightest of
                    // those upper bounds over the whole window is the highest this
                    // cell's floor could be and still be ground; anything above it is
                    // something standing there. The symmetric lower bound gives the
                    // drop-off test.
                    //
                    // Note the window includes the cell itself (d = 0), so upper_bound
                    // is never above center and lower_bound never below it: both steps
                    // come out non-negative without a clamp.
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
                            static_cast<std::size_t>(neighbour_row - row +
                                                     window_cells) *
                            static_cast<std::size_t>(window_side);
                        for (int neighbour_col = col_begin; neighbour_col <= col_end;
                             ++neighbour_col)
                        {
                            const float neighbour =
                                elevation(neighbour_row, neighbour_col);
                            if (std::isnan(neighbour))
                            {
                                continue;
                            }
                            const float slack =
                                allowance[allowance_row +
                                          static_cast<std::size_t>(
                                              neighbour_col - col + window_cells)];
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

        void compute_structure(grid_map::GridMap& map,
                               const std::deque<BufferedCloud>& clouds,
                               const TerrainParameters& params)
        {
            const Eigen::MatrixXf& ground = map["ground"];
            Eigen::MatrixXf& height_above_ground = map["height_above_ground"];
            Eigen::MatrixXf& clearance = map["clearance"];

            const float height_cap = static_cast<float>(params.obstacle_height_cap_m);
            const float ground_margin = static_cast<float>(params.ground_margin_m);

            // A second pass over the same points, rather than reusing height_max from
            // accumulate_elevation: height_max is measured against nothing, and the two
            // things wanted here are both measured against the ground reference, which
            // only exists now. The tallest return BELOW the overhead cap is the
            // structure standing in the cell; the lowest return ABOVE the ground margin
            // is the ceiling over it.
            for (const auto& buffered : clouds)
            {
                for (const auto& point : buffered.points)
                {
                    grid_map::Index index;
                    if (!map.getIndex(grid_map::Position(point.x(), point.y()), index))
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
                        cell_height = std::isnan(cell_height)
                                          ? above
                                          : std::max(cell_height, above);
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

        void compute_border(grid_map::GridMap& map, const TerrainParameters& params)
        {
            const Eigen::MatrixXf& point_count = map["point_count"];
            Eigen::MatrixXf& border = map["border"];
            border = (point_count.array() < static_cast<float>(params.min_points_per_cell))
                         .cast<float>()
                         .matrix();
        }

        void prune_small_obstacles(grid_map::GridMap& map, int min_obstacle_cells)
        {
            if (min_obstacle_cells <= 1)
            {
                return;
            }

            Eigen::MatrixXf& obstacle = map["obstacle"];
            const int rows = map.getSize()(0);
            const int cols = map.getSize()(1);

            // 8-connected flood fill: a sparsely sampled boulder can leave a cell
            // touching its neighbour only at a corner, and splitting it in two would be
            // the filter working against itself.
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

                    if (component.size() < static_cast<std::size_t>(min_obstacle_cells))
                    {
                        for (const auto& [component_row, component_col] : component)
                        {
                            obstacle(component_row, component_col) = 0.0f;
                        }
                    }
                }
            }
        }

        void compute_obstacle(grid_map::GridMap& map, const TerrainParameters& params)
        {
            const Eigen::MatrixXf& step_up = map["step_up"];
            const Eigen::MatrixXf& step_down = map["step_down"];
            const Eigen::MatrixXf& height_above_ground = map["height_above_ground"];
            const Eigen::MatrixXf& clearance = map["clearance"];
            const Eigen::MatrixXf& border = map["border"];
            Eigen::MatrixXf& obstacle = map["obstacle"];

            const float max_step_up = static_cast<float>(params.max_step_up_m);
            const float max_step_down = static_cast<float>(params.max_step_down_m);
            const float max_height = static_cast<float>(params.max_height_above_ground_m);
            const float min_clearance = static_cast<float>(params.min_clearance_m);

            const int rows = map.getSize()(0);
            const int cols = map.getSize()(1);
            for (int row = 0; row < rows; ++row)
            {
                for (int col = 0; col < cols; ++col)
                {
                    if (border(row, col) > 0.5f)
                    {
                        obstacle(row, col) =
                            params.treat_unknown_as_obstacle ? 1.0f : 0.0f;
                        continue;
                    }

                    const float step_up_value = step_up(row, col);
                    const float step_down_value = step_down(row, col);
                    const float height_value = height_above_ground(row, col);
                    const float clearance_value = clearance(row, col);

                    // Zero disables a test rather than making it fire on every cell,
                    // which is what the plain comparison would do for the two
                    // thresholds a non-negative quantity can equal.
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

            prune_small_obstacles(map, params.min_obstacle_cells);
        }
    }  // namespace

    std::vector<Eigen::Vector3f> filter_scan(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                             const Eigen::Isometry3f& sensor_to_global,
                                             const ScanFilter& filter)
    {
        const float min_range_squared =
            static_cast<float>(filter.min_range_m * filter.min_range_m);
        const float max_range_squared =
            static_cast<float>(filter.max_range_m * filter.max_range_m);
        const float max_sensor_height = static_cast<float>(filter.max_sensor_height_m);

        std::vector<Eigen::Vector3f> points;
        points.reserve(cloud.points.size());
        for (const auto& point : cloud.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z))
            {
                continue;
            }
            const float range_squared =
                point.x * point.x + point.y * point.y + point.z * point.z;
            if (range_squared < min_range_squared || range_squared > max_range_squared)
            {
                continue;
            }
            if (point.z > max_sensor_height)
            {
                continue;
            }
            points.push_back(sensor_to_global * Eigen::Vector3f(point.x, point.y, point.z));
        }
        return points;
    }

    void classify_terrain(grid_map::GridMap& map, const std::deque<BufferedCloud>& clouds,
                          const TerrainParameters& params)
    {
        map["elevation"].setConstant(NaN);
        map["height_max"].setConstant(NaN);
        map["point_count"].setConstant(0.0f);
        map["ground"].setConstant(NaN);
        map["step_up"].setConstant(NaN);
        map["step_down"].setConstant(NaN);
        map["height_above_ground"].setConstant(NaN);
        map["clearance"].setConstant(NaN);

        accumulate_elevation(map, clouds);
        compute_height_difference(map, params);
        compute_structure(map, clouds, params);
        compute_border(map, params);
        compute_obstacle(map, params);
    }

    void compute_inflation(grid_map::GridMap& map, const InflationParameters& params)
    {
        const Eigen::MatrixXf& obstacle = map["obstacle"];
        Eigen::MatrixXf& inflation = map["inflation"];

        const int rows = map.getSize()(0);
        const int cols = map.getSize()(1);
        const float resolution = static_cast<float>(map.getResolution());
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

        // Matches nav2's InflationLayer convention: lethal out to the robot's own
        // radius, then an exponential decay out to inflation_radius_m, then clear.
        const float inscribed_radius_cells =
            static_cast<float>(params.robot_radius_m) / resolution;
        const float inflation_radius_cells =
            static_cast<float>(params.inflation_radius_m) / resolution;
        const float scaling_factor = static_cast<float>(params.cost_scaling_factor);
        const float robot_radius = static_cast<float>(params.robot_radius_m);

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

    void compute_final_cost(grid_map::GridMap& map, bool treat_unknown_as_obstacle)
    {
        const Eigen::MatrixXf& obstacle = map["obstacle"];
        const Eigen::MatrixXf& inflation = map["inflation"];
        const Eigen::MatrixXf& border = map["border"];
        Eigen::MatrixXf& cost = map["cost"];

        const int rows = map.getSize()(0);
        const int cols = map.getSize()(1);
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (border(row, col) > 0.5f && !treat_unknown_as_obstacle)
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

    void to_occupancy_grid(const grid_map::GridMap& map, const std::string& layer,
                           double min_value, double max_value,
                           nav_msgs::msg::OccupancyGrid& occupancy_grid_out)
    {
        const double resolution = map.getResolution();
        const grid_map::Length length = map.getLength();
        const grid_map::Position center = map.getPosition();

        const int width =
            std::max(1, static_cast<int>(std::round(length.x() / resolution)));
        const int height =
            std::max(1, static_cast<int>(std::round(length.y() / resolution)));

        occupancy_grid_out.header.frame_id = map.getFrameId();
        occupancy_grid_out.info.resolution = static_cast<float>(resolution);
        occupancy_grid_out.info.width = static_cast<uint32_t>(width);
        occupancy_grid_out.info.height = static_cast<uint32_t>(height);
        occupancy_grid_out.info.origin.position.x = center.x() - length.x() / 2.0;
        occupancy_grid_out.info.origin.position.y = center.y() - length.y() / 2.0;
        occupancy_grid_out.info.origin.position.z = 0.0;
        occupancy_grid_out.info.origin.orientation.w = 1.0;

        occupancy_grid_out.data.assign(
            static_cast<size_t>(width) * static_cast<size_t>(height), -1);

        const Eigen::MatrixXf& values = map[layer];
        const double range = max_value - min_value;

        // Iterate OccupancyGrid cells (a convention we fully control) rather than
        // grid_map's own row/column order, and ask grid_map's own getIndex() for the
        // matching cell each time -- that way this never has to know or reimplement
        // grid_map's internal index<->world convention, only trust the same lookup
        // already used everywhere else.
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const double world_x =
                    occupancy_grid_out.info.origin.position.x + (x + 0.5) * resolution;
                const double world_y =
                    occupancy_grid_out.info.origin.position.y + (y + 0.5) * resolution;

                grid_map::Index index;
                if (!map.getIndex(grid_map::Position(world_x, world_y), index))
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

}  // namespace local_traversability
