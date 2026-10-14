#pragma once

/// @file synthetic_scan.hpp
/// @brief A lidar on a mast over terrain given as a height function, for exercising
///        the terrain classifier without a rover, a simulator or a bag.

#include <cmath>
#include <deque>
#include <functional>
#include <grid_map_core/grid_map_core.hpp>

#include "local_traversability/local_traversability/terrain_analysis.hpp"

namespace synthetic_scan
{
    /// @brief Ground height at (x, y), in metres.
    using Terrain = std::function<float(float, float)>;

    /// @brief Height of the MID-360 above base_footprint on Perseus, in metres.
    constexpr float SENSOR_HEIGHT_M = 0.733f;

    /// @brief The beam pattern and range gates of the simulated sensor.
    /// @details The MID-360 is pitched 45 deg down on the mast, which puts its
    /// -7..+52 deg field of view at -52..+7 deg of elevation looking forward. It
    /// returns about 200 000 points a second over the full circle, so the one second
    /// of scans the classifier pools is roughly one point every third of a degree.
    struct Sensor
    {
        float azimuth_half_width_deg{60.0f};
        float min_elevation_deg{-52.0f};
        float max_elevation_deg{7.0f};
        float angular_step_deg{0.35f};
        float min_range_m{1.0f};
        float max_range_m{6.0f};
    };

    inline float deg_to_rad(float degrees)
    {
        return degrees * static_cast<float>(M_PI) / 180.0f;
    }

    /// @brief Flat ground at z = 0.
    inline Terrain flat()
    {
        return [](float, float)
        { return 0.0f; };
    }

    /// @brief A plane rising along +x at @p slope_deg, through z = 0 at x = 0.
    inline Terrain slope(float slope_deg)
    {
        const float gradient = std::tan(deg_to_rad(slope_deg));
        return [gradient](float x, float)
        { return gradient * x; };
    }

    /// @brief Smooth undulation: the lumpy bed of the arena without anything dug in it.
    inline Terrain undulating(float amplitude_m, float wavelength_m)
    {
        const float k = 2.0f * static_cast<float>(M_PI) / wavelength_m;
        return [amplitude_m, k](float x, float y)
        { return amplitude_m * std::sin(k * x) * std::cos(k * y); };
    }

    /// @brief Adds a bowl-shaped crater (deepest at the centre, a paraboloid out to
    ///        the rim) to @p base.
    inline Terrain with_bowl_crater(Terrain base, float centre_x, float centre_y,
                                    float width_m, float depth_m)
    {
        const float radius = width_m / 2.0f;
        return [=](float x, float y)
        {
            const float r = std::hypot(x - centre_x, y - centre_y);
            const float dug =
                r < radius ? depth_m * (1.0f - (r / radius) * (r / radius)) : 0.0f;
            return base(x, y) - dug;
        };
    }

    /// @brief Adds a steep-sided crater (flat floor over the inner 60 % of the radius,
    ///        straight walls up to the rim) to @p base.
    inline Terrain with_steep_crater(Terrain base, float centre_x, float centre_y,
                                     float width_m, float depth_m)
    {
        const float radius = width_m / 2.0f;
        const float floor_radius = 0.6f * radius;
        return [=](float x, float y)
        {
            const float r = std::hypot(x - centre_x, y - centre_y);
            float dug = 0.0f;
            if (r < floor_radius)
            {
                dug = depth_m;
            }
            else if (r < radius)
            {
                dug = depth_m * (radius - r) / (radius - floor_radius);
            }
            return base(x, y) - dug;
        };
    }

    /// @brief Adds a boulder (a dome) to @p base.
    inline Terrain with_boulder(Terrain base, float centre_x, float centre_y,
                                float width_m, float height_m)
    {
        const float radius = width_m / 2.0f;
        return [=](float x, float y)
        {
            const float r = std::hypot(x - centre_x, y - centre_y);
            const float raised =
                r < radius ? height_m * std::sqrt(1.0f - (r / radius) * (r / radius))
                           : 0.0f;
            return base(x, y) + raised;
        };
    }

    /// @brief Casts one scan's worth of beams from a sensor at (@p sensor_x,
    ///        @p sensor_y), looking along +x, and returns where they land.
    /// @details Each beam is marched forward until it goes under the terrain, then the
    /// crossing is refined by bisection. Beams that never land, or land outside the
    /// range gates, give no point -- as on the real sensor.
    inline local_traversability::BufferedCloud scan(const Terrain& terrain,
                                                    const Sensor& sensor = {},
                                                    float sensor_x = 0.0f,
                                                    float sensor_y = 0.0f)
    {
        constexpr float MARCH_STEP_M = 0.01f;
        constexpr int BISECTION_STEPS = 10;

        local_traversability::BufferedCloud cloud;
        cloud.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        cloud.sensor_origin = Eigen::Vector3f(
            sensor_x, sensor_y, terrain(sensor_x, sensor_y) + SENSOR_HEIGHT_M);

        for (float azimuth_deg = -sensor.azimuth_half_width_deg;
             azimuth_deg <= sensor.azimuth_half_width_deg;
             azimuth_deg += sensor.angular_step_deg)
        {
            for (float elevation_deg = sensor.min_elevation_deg;
                 elevation_deg <= sensor.max_elevation_deg;
                 elevation_deg += sensor.angular_step_deg)
            {
                const float azimuth = deg_to_rad(azimuth_deg);
                const float elevation = deg_to_rad(elevation_deg);
                const Eigen::Vector3f direction(std::cos(elevation) * std::cos(azimuth),
                                                std::cos(elevation) * std::sin(azimuth),
                                                std::sin(elevation));

                const auto above = [&](float range)
                {
                    const Eigen::Vector3f at = cloud.sensor_origin + direction * range;
                    return at.z() - terrain(at.x(), at.y());
                };

                for (float range = MARCH_STEP_M; range <= sensor.max_range_m;
                     range += MARCH_STEP_M)
                {
                    if (above(range) > 0.0f)
                    {
                        continue;
                    }
                    float clear = range - MARCH_STEP_M;
                    float buried = range;
                    for (int i = 0; i < BISECTION_STEPS; ++i)
                    {
                        const float middle = 0.5f * (clear + buried);
                        (above(middle) > 0.0f ? clear : buried) = middle;
                    }
                    if (buried >= sensor.min_range_m)
                    {
                        cloud.points.push_back(cloud.sensor_origin + direction * buried);
                    }
                    break;
                }
            }
        }
        return cloud;
    }

    /// @brief local_traversability's thresholds from navigation.yaml, with the hole
    ///        test at its starting values.
    inline local_traversability::TerrainParameters local_parameters()
    {
        local_traversability::TerrainParameters params{};
        params.ground_window_m = 0.5;
        params.max_slope_deg = 40.0;
        params.max_step_up_m = 0.2;
        params.max_step_down_m = 0.15;
        params.max_height_above_ground_m = 0.2;
        params.obstacle_height_cap_m = 1.0;
        params.ground_margin_m = 0.10;
        params.min_clearance_m = 0.0;
        params.min_points_per_cell = 1;
        params.min_obstacle_cells = 2;
        params.treat_unknown_as_obstacle = false;
        params.hole_depth_m = 0.08;
        params.hole_min_rays = 3;
        params.hole_max_range_m = 4.0;
        params.hole_reference_block_m = 0.5;
        return params;
    }

    /// @brief An empty 8 m grid at 0.1 m, centred on the origin: local_traversability's
    ///        window, with the layers compute_inflation and compute_final_cost add.
    inline grid_map::GridMap make_map()
    {
        std::vector<std::string> layers = local_traversability::TERRAIN_LAYERS;
        layers.emplace_back("inflation");
        layers.emplace_back("cost");
        grid_map::GridMap map(layers);
        map.setGeometry(grid_map::Length(8.0, 8.0), 0.1, grid_map::Position(0.0, 0.0));
        map.setFrameId("odom");
        return map;
    }

    /// @brief Scans @p terrain once from the origin and classifies it.
    inline grid_map::GridMap classify(
        const Terrain& terrain,
        const local_traversability::TerrainParameters& params = local_parameters(),
        const Sensor& sensor = {})
    {
        grid_map::GridMap map = make_map();
        std::deque<local_traversability::BufferedCloud> clouds;
        clouds.push_back(scan(terrain, sensor));
        local_traversability::classify_terrain(map, clouds, params);
        return map;
    }

}  // namespace synthetic_scan
