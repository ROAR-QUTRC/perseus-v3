/// @file crater_detection.cpp
/// @brief Implementation of the crater detector.

#include "global_traversability/global_traversability/crater_detection.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace global_traversability
{
    namespace
    {
        constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

        /// @brief Sectors the ring is split into to check a cell is surrounded.
        constexpr int SECTORS = 8;

        /// @brief One ring of neighbour offsets around a cell.
        struct Ring
        {
            std::vector<int> d_row;
            std::vector<int> d_col;
            /// @brief Sector (0..SECTORS-1) of each offset.
            std::vector<std::uint8_t> sector;
            /// @brief Mean distance of the offsets from the center, in metres.
            float mean_radius_m{0.0f};
        };

        std::vector<Ring> build_rings(const std::vector<double>& edges, double resolution)
        {
            std::vector<Ring> rings;
            for (std::size_t i = 0; i + 1 < edges.size(); ++i)
            {
                const double inner = edges[i];
                const double outer = edges[i + 1];
                const int reach = static_cast<int>(std::ceil(outer / resolution));
                Ring ring;
                double distance_sum = 0.0;
                for (int d_row = -reach; d_row <= reach; ++d_row)
                {
                    for (int d_col = -reach; d_col <= reach; ++d_col)
                    {
                        const double distance =
                            std::hypot(static_cast<double>(d_row),
                                       static_cast<double>(d_col)) *
                            resolution;
                        if (distance < inner || distance > outer)
                        {
                            continue;
                        }
                        const double angle =
                            std::atan2(static_cast<double>(d_row),
                                       static_cast<double>(d_col)) +
                            M_PI;
                        ring.d_row.push_back(d_row);
                        ring.d_col.push_back(d_col);
                        ring.sector.push_back(static_cast<std::uint8_t>(
                            static_cast<int>(angle / (2.0 * M_PI / SECTORS)) % SECTORS));
                        distance_sum += distance;
                    }
                }
                if (ring.d_row.empty())
                {
                    continue;
                }
                ring.mean_radius_m = static_cast<float>(
                    distance_sum / static_cast<double>(ring.d_row.size()));
                rings.push_back(std::move(ring));
            }
            return rings;
        }

        /// @brief Whether the occupied sectors in @p mask include two on roughly
        ///        opposite sides (at least 3 of 8 sectors, ~135 deg, apart).
        bool surrounds(unsigned mask)
        {
            for (int s = 0; s < SECTORS; ++s)
            {
                if ((mask & (1u << s)) == 0)
                {
                    continue;
                }
                const unsigned opposite = (1u << ((s + 3) % SECTORS)) |
                                          (1u << ((s + 4) % SECTORS)) |
                                          (1u << ((s + 5) % SECTORS));
                if ((mask & opposite) != 0)
                {
                    return true;
                }
            }
            return false;
        }

        float median(std::vector<float>& values)
        {
            const std::size_t middle = values.size() / 2;
            std::nth_element(values.begin(), values.begin() + middle, values.end());
            const float upper = values[middle];
            if (values.size() % 2 == 1)
            {
                return upper;
            }
            const float lower = *std::max_element(values.begin(), values.begin() + middle);
            return 0.5f * (lower + upper);
        }
    }  // namespace

    void detect_craters(grid_map::GridMap& window, const CraterParameters& params)
    {
        const Eigen::MatrixXf& elevation = window["elevation"];
        const Eigen::MatrixXf& border = window["border"];
        const Eigen::MatrixXf& obstacle = window["obstacle"];
        const Eigen::MatrixXf& height = window["height_above_ground"];
        Eigen::MatrixXf& crater = window["crater"];
        Eigen::MatrixXf& crater_depth = window["crater_depth"];
        crater.setConstant(NaN);
        crater_depth.setConstant(NaN);
        const float structure_height = static_cast<float>(params.structure_height_m);
        // An obstacle cell is a structure when it stands taller than structure_height, or
        // when it has no height at all (nothing to say it is only a rim).
        const auto is_structure = [&](int cell_row, int cell_col)
        {
            if (!(obstacle(cell_row, cell_col) > 0.5f))
            {
                return false;
            }
            const float cell_height = height(cell_row, cell_col);
            return std::isnan(cell_height) || cell_height > structure_height;
        };

        const std::vector<Ring> rings = build_rings(params.ring_edges_m, window.getResolution());
        if (rings.empty())
        {
            return;
        }

        const int rows = window.getSize()(0);
        const int cols = window.getSize()(1);
        const float threshold =
            static_cast<float>(std::tan(params.min_slope_deg * M_PI / 180.0));

        std::size_t most_cells = 0;
        for (const auto& ring : rings)
        {
            most_cells = std::max(most_cells, ring.d_row.size());
        }
        std::vector<float> values;
        values.reserve(most_cells);

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                const float center = elevation(row, col);
                if (border(row, col) > 0.5f || std::isnan(center) || is_structure(row, col))
                {
                    continue;
                }

                float steepest = -std::numeric_limits<float>::infinity();
                float steepest_depth = 0.0f;
                for (const auto& ring : rings)
                {
                    values.clear();
                    unsigned sectors = 0;
                    float sector_sum[SECTORS] = {};
                    int sector_count[SECTORS] = {};
                    for (std::size_t i = 0; i < ring.d_row.size(); ++i)
                    {
                        const int neighbour_row = row + ring.d_row[i];
                        const int neighbour_col = col + ring.d_col[i];
                        if (neighbour_row < 0 || neighbour_row >= rows ||
                            neighbour_col < 0 || neighbour_col >= cols)
                        {
                            continue;
                        }
                        const float neighbour = elevation(neighbour_row, neighbour_col);
                        if (std::isnan(neighbour) ||
                            border(neighbour_row, neighbour_col) > 0.5f ||
                            is_structure(neighbour_row, neighbour_col))
                        {
                            continue;
                        }
                        values.push_back(neighbour);
                        sectors |= 1u << ring.sector[i];
                        sector_sum[ring.sector[i]] += neighbour;
                        ++sector_count[ring.sector[i]];
                    }

                    const double cover = static_cast<double>(values.size()) /
                                         static_cast<double>(ring.d_row.size());
                    if (static_cast<int>(values.size()) < params.min_ring_cells ||
                        cover < params.min_ring_cover || !surrounds(sectors))
                    {
                        continue;
                    }
                    // Every side it was seen from has to rise, or this is a bank's foot.
                    const float needed_rise = 0.5f * threshold * ring.mean_radius_m;
                    int observed_sectors = 0;
                    int rising_sectors = 0;
                    for (int sector = 0; sector < SECTORS; ++sector)
                    {
                        if (sector_count[sector] == 0)
                        {
                            continue;
                        }
                        ++observed_sectors;
                        if (sector_sum[sector] / static_cast<float>(sector_count[sector]) -
                                center >=
                            needed_rise)
                        {
                            ++rising_sectors;
                        }
                    }
                    if (static_cast<double>(rising_sectors) <
                        params.min_rim_fraction * static_cast<double>(observed_sectors))
                    {
                        continue;
                    }

                    const float depth = median(values) - center;
                    if (depth / ring.mean_radius_m > steepest)
                    {
                        steepest = depth / ring.mean_radius_m;
                        steepest_depth = depth;
                    }
                }

                if (std::isfinite(steepest))
                {
                    crater(row, col) = steepest >= threshold ? 1.0f : 0.0f;
                    crater_depth(row, col) = steepest_depth;
                }
            }
        }
    }

    void demote_crater_obstacles(grid_map::GridMap& window, const CraterParameters& params)
    {
        const int reach =
            static_cast<int>(std::lround(params.obstacle_margin_m / window.getResolution()));
        const Eigen::MatrixXf& crater = window["crater"];
        const Eigen::MatrixXf& height = window["height_above_ground"];
        const Eigen::MatrixXf& crater_depth = window["crater_depth"];
        Eigen::MatrixXf& obstacle = window["obstacle"];
        const int rows = window.getSize()(0);
        const int cols = window.getSize()(1);
        const float structure_height = static_cast<float>(params.structure_height_m);
        // Noise on the rim's height and on the depth estimate, in metres.
        constexpr float NOISE_ALLOWANCE = 0.02f;

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (!(crater(row, col) > 0.5f))
                {
                    continue;
                }
                const float max_height =
                    std::min(structure_height, crater_depth(row, col) + NOISE_ALLOWANCE);
                for (int d_row = -reach; d_row <= reach; ++d_row)
                {
                    for (int d_col = -reach; d_col <= reach; ++d_col)
                    {
                        const int target_row = row + d_row;
                        const int target_col = col + d_col;
                        if (target_row < 0 || target_row >= rows || target_col < 0 ||
                            target_col >= cols || d_row * d_row + d_col * d_col > reach * reach)
                        {
                            continue;
                        }
                        const float target_height = height(target_row, target_col);
                        if (obstacle(target_row, target_col) > 0.5f &&
                            !std::isnan(target_height) && target_height <= max_height)
                        {
                            obstacle(target_row, target_col) = 0.0f;
                        }
                    }
                }
            }
        }
    }

    namespace
    {
        using Mask = Eigen::Matrix<std::uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

        std::vector<std::pair<int, int>> disc_offsets(int reach)
        {
            std::vector<std::pair<int, int>> disc;
            for (int d_row = -reach; d_row <= reach; ++d_row)
            {
                for (int d_col = -reach; d_col <= reach; ++d_col)
                {
                    if (d_row * d_row + d_col * d_col <= reach * reach)
                    {
                        disc.emplace_back(d_row, d_col);
                    }
                }
            }
            return disc;
        }

        /// @brief Dilates the set cells of @p mask_layer by @p disc into @p dilated and
        ///        lists every cell it set.
        /// @details From the set cells only: the mask is a handful of small blobs on an
        /// arena-sized grid, so walking every cell would be nearly all wasted work.
        std::vector<std::pair<int, int>> dilate(
            const Eigen::MatrixXf& mask_layer,
            const std::vector<std::pair<int, int>>& disc, Mask& dilated)
        {
            const int rows = static_cast<int>(mask_layer.rows());
            const int cols = static_cast<int>(mask_layer.cols());
            dilated = Mask::Zero(rows, cols);
            std::vector<std::pair<int, int>> reached;
            for (int row = 0; row < rows; ++row)
            {
                for (int col = 0; col < cols; ++col)
                {
                    if (!(mask_layer(row, col) > 0.5f))
                    {
                        continue;
                    }
                    for (const auto& [d_row, d_col] : disc)
                    {
                        const int target_row = row + d_row;
                        const int target_col = col + d_col;
                        if (target_row < 0 || target_row >= rows || target_col < 0 ||
                            target_col >= cols || dilated(target_row, target_col) != 0)
                        {
                            continue;
                        }
                        dilated(target_row, target_col) = 1;
                        reached.emplace_back(target_row, target_col);
                    }
                }
            }
            return reached;
        }
    }  // namespace

    void dilate_mask(grid_map::GridMap& map, const std::string& layer, double radius_m)
    {
        const int reach = static_cast<int>(std::lround(radius_m / map.getResolution()));
        if (reach < 1 || !map.exists(layer))
        {
            return;
        }
        Eigen::MatrixXf& mask = map[layer];
        Mask dilated;
        for (const auto& [row, col] : dilate(mask, disc_offsets(reach), dilated))
        {
            mask(row, col) = 1.0f;
        }
    }

    void close_mask(grid_map::GridMap& map, const std::string& layer, double radius_m)
    {
        const int reach = static_cast<int>(std::lround(radius_m / map.getResolution()));
        if (reach < 1 || !map.exists(layer))
        {
            return;
        }

        Eigen::MatrixXf& mask = map[layer];
        const int rows = static_cast<int>(mask.rows());
        const int cols = static_cast<int>(mask.cols());
        const auto disc = disc_offsets(reach);

        Mask dilated;
        const auto reached = dilate(mask, disc, dilated);

        // Erode: a dilated cell stays only if its whole disc is dilated. Off-grid counts
        // as dilated, so the closing never eats into the map's own edge.
        for (const auto& [row, col] : reached)
        {
            bool all_set = true;
            for (const auto& [d_row, d_col] : disc)
            {
                const int neighbour_row = row + d_row;
                const int neighbour_col = col + d_col;
                if (neighbour_row < 0 || neighbour_row >= rows || neighbour_col < 0 ||
                    neighbour_col >= cols)
                {
                    continue;
                }
                if (dilated(neighbour_row, neighbour_col) == 0)
                {
                    all_set = false;
                    break;
                }
            }
            if (all_set)
            {
                mask(row, col) = 1.0f;
            }
        }
    }

}  // namespace global_traversability
