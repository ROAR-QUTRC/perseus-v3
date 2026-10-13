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

        /// @brief Fewest ground-like cells a block needs before its median is trusted.
        constexpr std::size_t MIN_REFERENCE_CELLS = 3;
        /// @brief Fewest valid blocks, out of a block and its eight neighbours, needed
        ///        before the regional ground is defined there. A plane needs three.
        constexpr std::size_t MIN_REFERENCE_BLOCKS = 3;
        /// @brief Most blocks that may be dropped from one plane fit as outliers.
        /// @details Two, because that is how many blocks a guidebook crater can drag
        /// down: it fills one, or sits across the edge between two.
        constexpr int REFERENCE_MAX_OUTLIERS = 2;
        /// @brief How far a block may sit from the fitted plane, as a fraction of
        ///        hole_depth_m, before it is dropped as an outlier.
        constexpr double REFERENCE_OUTLIER_FRACTION = 0.5;
        /// @brief Ridge penalty on the fitted slopes, in m^2. See fit_plane().
        constexpr double REFERENCE_SLOPE_RIDGE = 0.05;
        /// @brief Radius around the sensor, in m, whose block samples set the height the
        ///        ground is grown outwards from. See compute_ground_reference() step 2.
        constexpr double REFERENCE_SEED_RADIUS_M = 2.5;
        /// @brief Steepest ground the reference will follow from one block to the next,
        ///        as rise over run: tan(20 deg). Steeper than any bed the rover drives.
        constexpr double REFERENCE_MAX_SLOPE = 0.36;
        /// @brief Height two neighbouring block samples may differ by on level ground, m.
        constexpr double REFERENCE_STEP_MARGIN_M = 0.05;
        /// @brief How many blocks away a sample may be and still count as a neighbour
        ///        when growing the ground. Two bridges the one-block shadow of a rock.
        constexpr int REFERENCE_REACH_BLOCKS = 2;
        /// @brief Distance between samples when walking back along a ray, as a fraction
        ///        of a cell. Half a cell cannot step over one.
        constexpr float RAY_STEP_CELLS = 0.5f;
        /// @brief Fraction of hole_depth_m a ray must stay below the regional ground to
        ///        keep marking cells on its way back to the sensor.
        /// @details Lower than the 1.0 that starts a walk, on purpose. The return that
        /// starts it is the deepest point of the ray; every cell nearer the sensor sees
        /// the same ray higher up, so holding them to the full depth would mark only
        /// the far wall and leave the shadowed middle of the crater, where the rover
        /// would actually fall in, unmarked. Not zero either: the regional ground is
        /// only good to a few centimetres, and at zero that noise decides how far past
        /// the near rim the mark runs.
        constexpr float RAY_WALK_DEPTH_FRACTION = 0.5f;

        /// @brief Middle value of @p values, which it reorders. Must not be empty.
        float median(std::vector<float>& values)
        {
            const auto middle =
                values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
            std::nth_element(values.begin(), middle, values.end());
            return *middle;
        }

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

        /// @brief One block's ground sample: where its ground-like cells are, on
        ///        average, and their median height.
        struct BlockSample
        {
            Eigen::Vector3d position{Eigen::Vector3d::Zero()};
            bool valid{false};
        };

        /// @brief The regional ground around one block, as a plane
        ///        z = height + slope . (xy - anchor).
        struct BlockPlane
        {
            Eigen::Vector2d anchor{Eigen::Vector2d::Zero()};
            double height{0.0};
            Eigen::Vector2d slope{Eigen::Vector2d::Zero()};
            bool valid{false};

            double at(const grid_map::Position& position) const
            {
                return height + slope.dot(position - anchor);
            }
        };

        /// @brief Least-squares plane through @p samples, about @p anchor.
        /// @details The two slope terms carry a ridge penalty. Nine samples spread over
        /// 1.5 m put 1.5 m^2 against its 0.05, so it costs a full fit 3 % of the slope;
        /// samples that lie along one line, or nearly, put a few cm^2 against it, and
        /// there it picks the plane that does not tilt across the line. Plain least
        /// squares has no answer on a line and a wild one near it: at the edge of the
        /// lidar's view three part-filled blocks a few cm out of line gave a slope
        /// that put the reference 0.3 m up half a metre away (Gazebo, beside the
        /// wall).
        BlockPlane fit_plane(const std::vector<Eigen::Vector3d>& samples,
                             const Eigen::Vector2d& anchor)
        {
            Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
            Eigen::Vector3d moment = Eigen::Vector3d::Zero();
            for (const auto& sample : samples)
            {
                const Eigen::Vector3d row(1.0, sample.x() - anchor.x(),
                                          sample.y() - anchor.y());
                normal += row * row.transpose();
                moment += row * sample.z();
            }
            normal(1, 1) += REFERENCE_SLOPE_RIDGE;
            normal(2, 2) += REFERENCE_SLOPE_RIDGE;

            const Eigen::Vector3d solution = normal.ldlt().solve(moment);
            BlockPlane plane;
            plane.anchor = anchor;
            plane.height = solution(0);
            plane.slope = solution.tail<2>();
            plane.valid = true;
            return plane;
        }

        void compute_ground_reference(grid_map::GridMap& map,
                                      const std::deque<BufferedCloud>& clouds,
                                      const TerrainParameters& params)
        {
            const Eigen::MatrixXf& elevation = map["elevation"];
            const Eigen::MatrixXf& step_up = map["step_up"];
            const Eigen::MatrixXf& height_above_ground = map["height_above_ground"];
            Eigen::MatrixXf& ground_ref = map["ground_ref"];

            const int rows = map.getSize()(0);
            const int cols = map.getSize()(1);
            const int block_cells = std::max(
                1, static_cast<int>(std::round(params.hole_reference_block_m /
                                               map.getResolution())));
            const int block_rows = (rows + block_cells - 1) / block_cells;
            const int block_cols = (cols + block_cells - 1) / block_cells;
            const auto block_index = [block_cols](int block_row, int block_col)
            {
                return static_cast<std::size_t>(block_row) *
                           static_cast<std::size_t>(block_cols) +
                       static_cast<std::size_t>(block_col);
            };
            const std::size_t block_count = static_cast<std::size_t>(block_rows) *
                                            static_cast<std::size_t>(block_cols);

            const float max_step_up = static_cast<float>(params.max_step_up_m);
            const float max_height = static_cast<float>(params.max_height_above_ground_m);
            const double outlier_m = params.hole_depth_m * REFERENCE_OUTLIER_FRACTION;

            // Ground is something the lidar looks down on. A ceiling, a tent roof or
            // the top of a bench is as flat as the bed and passes every test below,
            // and one of them taken as ground puts the reference metres up and turns
            // every ray under it into a hole (Gazebo arena: 600 cells under the
            // building's roof, seen behind the rover at 4-5 m up). So a cell at or
            // above the lowest sensor position in the buffer cannot be a sample. The
            // cost is on a steep climb, where the bed ahead rises above the lidar
            // within range: there is no reference there, and so no hole test.
            float lowest_sensor_z = std::numeric_limits<float>::infinity();
            for (const BufferedCloud& buffered : clouds)
            {
                lowest_sensor_z = std::min(lowest_sensor_z, buffered.sensor_origin.z());
            }

            // WHY NOT THE "ground" LAYER. That one is the tightest bound the cells within
            // ground_window_m allow, so it follows the floor down into anything wider
            // than the window's slope allowance can bridge -- which is the point of it
            // for rocks, and exactly wrong here: a reference that dips into the crater
            // reports the crater as flat. This one answers a different question, "where
            // would the ground be if nothing were dug out of it", so it has to be taken
            // over an area a crater cannot dominate, by statistics a crater cannot drag.
            //
            // Step 1. One sample per block: the median height of its ground-like cells,
            // placed at the average position of those cells. A cell the step-up or
            // height test would flag is a rock or a wall, and a wall face inside a
            // block otherwise lifts its median by the height of the wall. The position
            // matters as much as the height: a block at the edge of what the lidar
            // sees is only part filled, so its median belongs to the part that was
            // seen, not to the block's center.
            std::vector<BlockSample> samples(block_count);
            std::vector<float> heights;
            for (int block_row = 0; block_row < block_rows; ++block_row)
            {
                for (int block_col = 0; block_col < block_cols; ++block_col)
                {
                    heights.clear();
                    grid_map::Position position_sum = grid_map::Position::Zero();
                    const int row_end = std::min(rows, (block_row + 1) * block_cells);
                    const int col_end = std::min(cols, (block_col + 1) * block_cells);
                    for (int row = block_row * block_cells; row < row_end; ++row)
                    {
                        for (int col = block_col * block_cells; col < col_end; ++col)
                        {
                            const float floor = elevation(row, col);
                            if (std::isnan(floor) || floor >= lowest_sensor_z ||
                                step_up(row, col) > max_step_up ||
                                height_above_ground(row, col) > max_height)
                            {
                                continue;
                            }
                            grid_map::Position position;
                            map.getPosition(grid_map::Index(row, col), position);
                            position_sum += position;
                            heights.push_back(floor);
                        }
                    }
                    if (heights.size() < MIN_REFERENCE_CELLS)
                    {
                        continue;
                    }
                    BlockSample& sample = samples[block_index(block_row, block_col)];
                    const grid_map::Position mean =
                        position_sum / static_cast<double>(heights.size());
                    sample.position =
                        Eigen::Vector3d(mean.x(), mean.y(), median(heights));
                    sample.valid = true;
                }
            }

            // Step 2. Keep only the samples that are the ground the rover stands on.
            // Flat is not enough to be ground: the top of the arena wall is flat, 0.7 m
            // up and just under the lidar, and so is a bench or a crate. In Gazebo the
            // wall top alone raised the reference beside the wall by 0.3 m and put 100
            // hole cells on good ground. What tells them apart is that ground is
            // joined to the ground under the rover without a jump. So: take the median
            // of the samples near the sensor as the height to start from, then grow
            // outwards, accepting a sample only if an already accepted one within
            // REFERENCE_REACH_BLOCKS is at nearly its height. "Nearly" is one block of
            // the steepest slope plus a margin, whatever the gap between the two, so a
            // gap in the samples cannot be used to climb onto a wall. The floor of a
            // deep crater is left out by the same rule, which is what step 3 wants.
            if (!clouds.empty())
            {
                const Eigen::Vector2d sensor_xy =
                    clouds.back().sensor_origin.head<2>().cast<double>();
                const double max_jump =
                    REFERENCE_MAX_SLOPE * params.hole_reference_block_m +
                    REFERENCE_STEP_MARGIN_M;

                heights.clear();
                for (const BlockSample& sample : samples)
                {
                    if (sample.valid && (sample.position.head<2>() - sensor_xy).norm() <=
                                            REFERENCE_SEED_RADIUS_M)
                    {
                        heights.push_back(static_cast<float>(sample.position.z()));
                    }
                }
                std::vector<bool> accepted(block_count, false);
                std::vector<std::size_t> frontier;
                if (!heights.empty())
                {
                    const double seed_height = static_cast<double>(median(heights));
                    for (std::size_t i = 0; i < block_count; ++i)
                    {
                        const BlockSample& sample = samples[i];
                        if (sample.valid &&
                            (sample.position.head<2>() - sensor_xy).norm() <=
                                REFERENCE_SEED_RADIUS_M &&
                            std::abs(sample.position.z() - seed_height) <= max_jump)
                        {
                            accepted[i] = true;
                            frontier.push_back(i);
                        }
                    }
                }
                while (!frontier.empty())
                {
                    const std::size_t from = frontier.back();
                    frontier.pop_back();
                    const int from_row = static_cast<int>(from) / block_cols;
                    const int from_col = static_cast<int>(from) % block_cols;
                    for (int d_row = -REFERENCE_REACH_BLOCKS;
                         d_row <= REFERENCE_REACH_BLOCKS; ++d_row)
                    {
                        for (int d_col = -REFERENCE_REACH_BLOCKS;
                             d_col <= REFERENCE_REACH_BLOCKS; ++d_col)
                        {
                            const int to_row = from_row + d_row;
                            const int to_col = from_col + d_col;
                            if (to_row < 0 || to_row >= block_rows || to_col < 0 ||
                                to_col >= block_cols)
                            {
                                continue;
                            }
                            const std::size_t to = block_index(to_row, to_col);
                            if (accepted[to] || !samples[to].valid ||
                                std::abs(samples[to].position.z() -
                                         samples[from].position.z()) > max_jump)
                            {
                                continue;
                            }
                            accepted[to] = true;
                            frontier.push_back(to);
                        }
                    }
                }
                for (std::size_t i = 0; i < block_count; ++i)
                {
                    samples[i].valid = accepted[i];
                }
            }

            // Step 3. One plane per block, fitted to its own sample and its eight
            // neighbours'. A plane and not an average because the bed slopes, and an
            // average is only right in the middle of the samples it was taken over:
            // at the edge of the lidar's view, and at the edge of every shadow a rock
            // or a crater casts, the samples are all on one side, and an average there
            // is out by the slope times the distance to them.
            //
            // The fit is then made robust by hand. A 0.5 m crater can fill one block,
            // or straddle two, and drag their medians down to its far wall; so the
            // sample furthest from the plane is dropped and the plane refitted, at
            // most REFERENCE_MAX_OUTLIERS times and only while one is further out than
            // outlier_m. Deterministic, unlike RANSAC, and nine points at most.
            std::vector<BlockPlane> planes(block_count);
            std::vector<Eigen::Vector3d> neighbourhood;
            for (int block_row = 0; block_row < block_rows; ++block_row)
            {
                for (int block_col = 0; block_col < block_cols; ++block_col)
                {
                    neighbourhood.clear();
                    for (int d_row = -1; d_row <= 1; ++d_row)
                    {
                        for (int d_col = -1; d_col <= 1; ++d_col)
                        {
                            const int neighbour_row = block_row + d_row;
                            const int neighbour_col = block_col + d_col;
                            if (neighbour_row < 0 || neighbour_row >= block_rows ||
                                neighbour_col < 0 || neighbour_col >= block_cols)
                            {
                                continue;
                            }
                            const BlockSample& sample =
                                samples[block_index(neighbour_row, neighbour_col)];
                            if (sample.valid)
                            {
                                neighbourhood.push_back(sample.position);
                            }
                        }
                    }
                    if (neighbourhood.size() < MIN_REFERENCE_BLOCKS)
                    {
                        continue;
                    }

                    Eigen::Vector2d anchor = Eigen::Vector2d::Zero();
                    for (const auto& sample : neighbourhood)
                    {
                        anchor += sample.head<2>();
                    }
                    anchor /= static_cast<double>(neighbourhood.size());

                    BlockPlane plane = fit_plane(neighbourhood, anchor);
                    for (int dropped = 0; dropped < REFERENCE_MAX_OUTLIERS &&
                                          neighbourhood.size() > MIN_REFERENCE_BLOCKS;
                         ++dropped)
                    {
                        std::size_t worst = 0;
                        double worst_error = 0.0;
                        for (std::size_t i = 0; i < neighbourhood.size(); ++i)
                        {
                            const double error = std::abs(
                                neighbourhood[i].z() -
                                plane.at(neighbourhood[i].head<2>()));
                            if (error > worst_error)
                            {
                                worst_error = error;
                                worst = i;
                            }
                        }
                        if (worst_error <= outlier_m)
                        {
                            break;
                        }
                        neighbourhood.erase(neighbourhood.begin() +
                                            static_cast<std::ptrdiff_t>(worst));
                        plane = fit_plane(neighbourhood, anchor);
                    }
                    planes[block_index(block_row, block_col)] = plane;
                }
            }

            // Step 4. Back to cells. Each cell reads the planes of the four blocks
            // around it at its own position and blends them by how close each block
            // is, so the reference has no seams where blocks meet. A block with no
            // plane is left out and the others reweighted.
            for (int row = 0; row < rows; ++row)
            {
                const float block_row_position =
                    (static_cast<float>(row) + 0.5f) / static_cast<float>(block_cells) -
                    0.5f;
                const int block_row_low = static_cast<int>(std::floor(block_row_position));
                const float row_weight_high =
                    block_row_position - static_cast<float>(block_row_low);
                for (int col = 0; col < cols; ++col)
                {
                    const float block_col_position =
                        (static_cast<float>(col) + 0.5f) /
                            static_cast<float>(block_cells) -
                        0.5f;
                    const int block_col_low =
                        static_cast<int>(std::floor(block_col_position));
                    const float col_weight_high =
                        block_col_position - static_cast<float>(block_col_low);

                    grid_map::Position position;
                    map.getPosition(grid_map::Index(row, col), position);

                    double weighted_sum = 0.0;
                    double weight_total = 0.0;
                    for (int corner_row = 0; corner_row <= 1; ++corner_row)
                    {
                        for (int corner_col = 0; corner_col <= 1; ++corner_col)
                        {
                            const BlockPlane& plane = planes[block_index(
                                std::clamp(block_row_low + corner_row, 0, block_rows - 1),
                                std::clamp(block_col_low + corner_col, 0,
                                           block_cols - 1))];
                            if (!plane.valid)
                            {
                                continue;
                            }
                            const double weight = static_cast<double>(
                                (corner_row == 1 ? row_weight_high
                                                 : 1.0f - row_weight_high) *
                                (corner_col == 1 ? col_weight_high
                                                 : 1.0f - col_weight_high));
                            weighted_sum += weight * plane.at(position);
                            weight_total += weight;
                        }
                    }
                    if (weight_total > 0.0)
                    {
                        ground_ref(row, col) =
                            static_cast<float>(weighted_sum / weight_total);
                    }
                }
            }
        }

        void compute_holes(grid_map::GridMap& map, const std::deque<BufferedCloud>& clouds,
                           const TerrainParameters& params)
        {
            const Eigen::MatrixXf& ground_ref = map["ground_ref"];
            Eigen::MatrixXf& hole_depth = map["hole_depth"];
            Eigen::MatrixXf& hole_rays = map["hole_rays"];
            Eigen::MatrixXf& hole = map["hole"];

            const float start_depth = static_cast<float>(params.hole_depth_m);
            const float walk_depth = start_depth * RAY_WALK_DEPTH_FRACTION;
            const float max_range_squared =
                static_cast<float>(params.hole_max_range_m * params.hole_max_range_m);
            const float step = static_cast<float>(map.getResolution()) * RAY_STEP_CELLS;

            for (const auto& buffered : clouds)
            {
                const Eigen::Vector3f& origin = buffered.sensor_origin;
                for (const auto& point : buffered.points)
                {
                    grid_map::Index index;
                    if (!map.getIndex(grid_map::Position(point.x(), point.y()), index))
                    {
                        continue;
                    }
                    // The cheap test, and the one almost every point fails: did this
                    // return land on the ground, or under where the ground should be?
                    const float reference = ground_ref(index(0), index(1));
                    if (std::isnan(reference) || reference - point.z() <= start_depth)
                    {
                        continue;
                    }

                    const Eigen::Vector3f to_sensor = origin - point;
                    // Beyond this the far-wall return of a real crater is no deeper than
                    // the regional ground is wrong by, so nothing out there is evidence.
                    if (to_sensor.head<2>().squaredNorm() > max_range_squared)
                    {
                        continue;
                    }
                    const float length = to_sensor.norm();
                    if (!(length > 0.0f))
                    {
                        continue;
                    }
                    const Eigen::Vector3f direction = to_sensor / length;

                    // THE RAY WALK. The beam travelled in a straight line from the
                    // sensor to this return and hit nothing on the way, so every point
                    // along it is known to be empty. Walk it backwards from the return:
                    // while the beam is still under the regional ground, the cell it is
                    // over has empty space where ground should be. It stops at the near
                    // rim, where the beam climbs back through ground level.
                    int last_row = -1;
                    int last_col = -1;
                    for (float travelled = 0.0f; travelled < length; travelled += step)
                    {
                        const Eigen::Vector3f sample = point + direction * travelled;
                        grid_map::Index cell;
                        if (!map.getIndex(grid_map::Position(sample.x(), sample.y()),
                                          cell))
                        {
                            break;
                        }
                        const float cell_reference = ground_ref(cell(0), cell(1));
                        if (std::isnan(cell_reference))
                        {
                            break;
                        }
                        const float below = cell_reference - sample.z();
                        if (below <= walk_depth)
                        {
                            break;
                        }

                        float& cell_depth = hole_depth(cell(0), cell(1));
                        cell_depth =
                            std::isnan(cell_depth) ? below : std::max(cell_depth, below);
                        // Once per ray per cell, however many samples fall in it:
                        // hole_min_rays counts independent beams, not step length.
                        if (cell(0) != last_row || cell(1) != last_col)
                        {
                            hole_rays(cell(0), cell(1)) += 1.0f;
                            last_row = cell(0);
                            last_col = cell(1);
                        }
                    }
                }
            }

            hole = (hole_rays.array() >= static_cast<float>(params.hole_min_rays))
                       .cast<float>()
                       .matrix();
        }

        void compute_border(grid_map::GridMap& map, const TerrainParameters& params)
        {
            const Eigen::MatrixXf& point_count = map["point_count"];
            const Eigen::MatrixXf& hole = map["hole"];
            Eigen::MatrixXf& border = map["border"];
            // A hole cell is observed, even with no return in it. The floor of a crater
            // is in the near rim's shadow, so by point count alone it is border --
            // "unknown", which this stack publishes as free and global_traversability
            // skips as no evidence. The rays that crossed it are the evidence.
            border = (point_count.array() < static_cast<float>(params.min_points_per_cell) &&
                      hole.array() < 0.5f)
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
            const Eigen::MatrixXf& hole = map["hole"];
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
                         clearance_value < min_clearance) ||
                        hole(row, col) > 0.5f;

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
        map["ground_ref"].setConstant(NaN);
        map["hole_depth"].setConstant(NaN);
        map["hole_rays"].setConstant(0.0f);
        map["hole"].setConstant(0.0f);

        accumulate_elevation(map, clouds);
        compute_height_difference(map, params);
        compute_structure(map, clouds, params);
        // The hole test comes after the two it borrows from (step_up and
        // height_above_ground say which cells are ground) and before the two it feeds:
        // a hole cell is not border, and it is an obstacle.
        if (params.hole_depth_m > 0.0)
        {
            compute_ground_reference(map, clouds, params);
            compute_holes(map, clouds, params);
        }
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
