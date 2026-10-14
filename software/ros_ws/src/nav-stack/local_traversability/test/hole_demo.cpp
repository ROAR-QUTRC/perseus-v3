/// @file hole_demo.cpp
/// @brief Prints what the hole test sees for one crater, as a picture in the terminal.
///
/// Usage: hole_demo [width_m] [depth_m] [distance_m] [hole_depth_m]
///
/// The lidar is at the left of each picture, 0.733 m up, looking right. Nothing here is
/// part of the nav stack; it exists so the test can be watched working.

#include <cstdio>
#include <cstdlib>

#include "synthetic_scan.hpp"

int main(int argc, char** argv)
{
    const float width = argc > 1 ? static_cast<float>(std::atof(argv[1])) : 0.5f;
    const float depth = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 0.45f;
    const float distance = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 2.5f;

    auto params = synthetic_scan::local_parameters();
    if (argc > 4)
    {
        params.hole_depth_m = std::atof(argv[4]);
    }

    const auto terrain = synthetic_scan::with_bowl_crater(synthetic_scan::flat(),
                                                          distance, 0.0f, width, depth);
    grid_map::GridMap map = synthetic_scan::classify(terrain, params);

    std::printf(
        "Bowl crater %.2f m wide, %.2f m deep, center %.2f m ahead. "
        "hole_depth_m %.2f, hole_min_rays %d.\n\n",
        width, depth, distance, params.hole_depth_m, params.hole_min_rays);

    // One row of cells straight ahead of the rover, through the crater's center.
    std::printf("Along the center line (y = 0):\n");
    std::printf("   x [m]  returns  lowest [m]  ground_ref [m]  rays below  verdict\n");
    for (float x = distance - 0.75f; x <= distance + 0.75f; x += 0.1f)
    {
        grid_map::Index index;
        if (!map.getIndex(grid_map::Position(x, 0.01), index))
        {
            continue;
        }
        const float lowest = map.at("elevation", index);
        const bool in_crater = std::abs(x - distance) < width / 2.0f;
        std::printf("  %6.2f  %7.0f  %10.2f  %14.2f  %10.0f  %s%s\n", x,
                    map.at("point_count", index), lowest, map.at("ground_ref", index),
                    map.at("hole_rays", index),
                    map.at("hole", index) > 0.5f ? "HOLE" : "-",
                    in_crater ? "   <- inside the crater" : "");
    }

    // Top view. One character per 0.1 m cell.
    std::printf("\nTop view, rover off the left edge, 0.1 m per character:\n");
    std::printf(
        "  O = hole (obstacle)   o = rays below ground, too few   "
        ". = ground seen   ' ' = no returns\n\n");
    for (float y = 0.75f; y >= -0.75f; y -= 0.1f)
    {
        std::printf("  ");
        for (float x = distance - 1.0f; x <= distance + 1.0f; x += 0.1f)
        {
            grid_map::Index index;
            char symbol = ' ';
            if (map.getIndex(grid_map::Position(x, y), index))
            {
                if (map.at("hole", index) > 0.5f)
                {
                    symbol = 'O';
                }
                else if (map.at("hole_rays", index) > 0.5f)
                {
                    symbol = 'o';
                }
                else if (map.at("point_count", index) > 0.5f)
                {
                    symbol = '.';
                }
            }
            std::printf("%c", symbol);
        }
        std::printf("\n");
    }

    int hole_cells = 0;
    int step_down_cells = 0;
    const auto& hole = map["hole"];
    const auto& step_down = map["step_down"];
    for (Eigen::Index i = 0; i < hole.size(); ++i)
    {
        hole_cells += hole(i) > 0.5f ? 1 : 0;
        step_down_cells += step_down(i) > static_cast<float>(params.max_step_down_m) ? 1 : 0;
    }
    std::printf("\nCells flagged: hole test %d, existing step_down test %d.\n", hole_cells,
                step_down_cells);
    return 0;
}
