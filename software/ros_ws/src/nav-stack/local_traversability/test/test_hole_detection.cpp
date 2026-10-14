/// @file test_hole_detection.cpp
/// @brief The hole (crater) test of the terrain classifier, against a synthetic lidar.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "synthetic_scan.hpp"

namespace
{
    using synthetic_scan::classify;
    using synthetic_scan::flat;
    using synthetic_scan::local_parameters;

    /// @brief Number of cells of @p layer above 0.5.
    int count_cells(const grid_map::GridMap& map, const std::string& layer)
    {
        return static_cast<int>((map[layer].array() > 0.5f).count());
    }

    /// @brief Number of cells of @p layer above 0.5 whose center is within
    ///        @p radius_m of (@p x, @p y), or outside it when @p inside is false.
    int count_cells_near(const grid_map::GridMap& map, const std::string& layer, float x,
                         float y, float radius_m, bool inside = true)
    {
        int count = 0;
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
        {
            if (map.at(layer, *it) <= 0.5f)
            {
                continue;
            }
            grid_map::Position position;
            map.getPosition(*it, position);
            const bool near = std::hypot(position.x() - x, position.y() - y) <= radius_m;
            count += near == inside ? 1 : 0;
        }
        return count;
    }

    /// @brief One crater and where it sits relative to the rover.
    struct CraterCase
    {
        const char* name;
        bool steep;
        float width_m;
        float depth_m;
        float distance_m;
        float bearing_deg;
    };

    void PrintTo(const CraterCase& crater, std::ostream* os)
    {
        *os << crater.name;
    }

    synthetic_scan::Terrain make_terrain(const CraterCase& crater, float& center_x,
                                         float& center_y)
    {
        const float bearing = synthetic_scan::deg_to_rad(crater.bearing_deg);
        center_x = crater.distance_m * std::cos(bearing);
        center_y = crater.distance_m * std::sin(bearing);
        return crater.steep
                   ? synthetic_scan::with_steep_crater(flat(), center_x, center_y,
                                                       crater.width_m, crater.depth_m)
                   : synthetic_scan::with_bowl_crater(flat(), center_x, center_y,
                                                      crater.width_m, crater.depth_m);
    }

    class DetectsCrater : public ::testing::TestWithParam<CraterCase>
    {
    };
}  // namespace

// ---- Ground that is not a hole must not become one ---------------------------------

TEST(HoleDetection, FlatGroundHasNoHoles)
{
    const auto map = classify(flat());
    EXPECT_EQ(count_cells(map, "hole"), 0);
    EXPECT_EQ(count_cells(map, "obstacle"), 0);
}

TEST(HoleDetection, SlopesHaveNoHoles)
{
    // Up and down, at the steepest the arena bed gets and a little beyond. This is the
    // test the interpolation in compute_ground_reference exists to pass.
    for (const float slope_deg : {-20.0f, -10.0f, 10.0f, 20.0f})
    {
        const auto map = classify(synthetic_scan::slope(slope_deg));
        EXPECT_EQ(count_cells(map, "hole"), 0) << "slope " << slope_deg << " deg";
    }
}

TEST(HoleDetection, UndulatingGroundHasNoHoles)
{
    // 10 cm of relief peak to trough over a 3 m wavelength.
    const auto map = classify(synthetic_scan::undulating(0.05f, 3.0f));
    EXPECT_EQ(count_cells(map, "hole"), 0);
}

TEST(HoleDetection, BoulderIsAnObstacleButNotAHole)
{
    // The ground behind a boulder is in its shadow, like the floor of a crater. Shadow
    // alone must not read as a hole: only rays seen under the ground do.
    const auto map = classify(synthetic_scan::with_boulder(flat(), 2.5f, 0.0f, 0.4f, 0.35f));
    EXPECT_EQ(count_cells(map, "hole"), 0);
    EXPECT_GT(count_cells_near(map, "obstacle", 2.5f, 0.0f, 0.4f), 0);
}

TEST(HoleDetection, CeilingBehindTheRoverIsNotGround)
{
    // The lidar is pitched down at the ground ahead, so behind the rover it looks up:
    // at a roof, 4.5 m overhead here. A roof is as flat as the bed. Taken as ground it
    // lifts the reference to its own height and every ray under it reads as a hole.
    // Found in the Gazebo arena, where it made 600 hole cells behind the rover.
    constexpr float CEILING_Z = 4.5f;
    auto cloud = synthetic_scan::scan(
        synthetic_scan::with_steep_crater(flat(), 2.5f, 0.0f, 0.5f, 0.45f));
    for (float x = -3.5f; x <= -0.5f; x += 0.03f)
    {
        for (float y = -3.0f; y <= 3.0f; y += 0.03f)
        {
            cloud.points.emplace_back(x, y, CEILING_Z);
        }
    }
    std::deque<local_traversability::BufferedCloud> clouds{cloud};
    grid_map::GridMap map = synthetic_scan::make_map();
    local_traversability::classify_terrain(map, clouds, local_parameters());

    EXPECT_EQ(count_cells_near(map, "hole", 2.5f, 0.0f, 0.5f, false), 0);
    EXPECT_GE(count_cells_near(map, "hole", 2.5f, 0.0f, 0.35f), 3);
}

TEST(HoleDetection, WallTopBesideTheRoverIsNotGround)
{
    // The arena wall: 0.72 m high, 0.3 m thick, its flat top just under the lidar.
    // Flat and below the sensor, so only its jump up from the bed says it is not
    // ground. Taken as ground it tilts the reference up towards the wall and the bed
    // along it reads as a hole. Found in the Gazebo arena: 100 cells beside the wall.
    constexpr float WALL_Y = 2.5f;
    constexpr float WALL_TOP_Z = 0.72f;
    auto cloud = synthetic_scan::scan(
        synthetic_scan::with_steep_crater(flat(), 2.5f, 0.0f, 0.5f, 0.45f));
    cloud.points.erase(std::remove_if(cloud.points.begin(), cloud.points.end(),
                                      [](const Eigen::Vector3f& point)
                                      { return point.y() > WALL_Y; }),
                       cloud.points.end());
    for (float x = 0.0f; x <= 3.9f; x += 0.03f)
    {
        for (float z = 0.0f; z <= WALL_TOP_Z; z += 0.03f)
        {
            cloud.points.emplace_back(x, WALL_Y + 0.02f, z);
        }
        for (float y = WALL_Y + 0.02f; y <= WALL_Y + 0.9f; y += 0.03f)
        {
            cloud.points.emplace_back(x, y, WALL_TOP_Z);
        }
    }
    std::deque<local_traversability::BufferedCloud> clouds{cloud};
    grid_map::GridMap map = synthetic_scan::make_map();
    local_traversability::classify_terrain(map, clouds, local_parameters());

    EXPECT_EQ(count_cells_near(map, "hole", 2.5f, 0.0f, 0.5f, false), 0);
    EXPECT_GE(count_cells_near(map, "hole", 2.5f, 0.0f, 0.35f), 3);
}

// ---- Craters must be found ---------------------------------------------------------

TEST_P(DetectsCrater, MarksItLethalAndNothingElse)
{
    const CraterCase crater = GetParam();
    float center_x = 0.0f;
    float center_y = 0.0f;
    const auto terrain = make_terrain(crater, center_x, center_y);

    grid_map::GridMap map = classify(terrain);
    local_traversability::compute_inflation(map, {0.5, 0.8, 3.2});
    local_traversability::compute_final_cost(map, false);

    const float radius = crater.width_m / 2.0f;

    // Found: a blob on the crater, not a stray cell.
    EXPECT_GE(count_cells_near(map, "hole", center_x, center_y, radius + 0.1f), 3);
    // Nothing invented: no hole cell further than a cell or two beyond the rim.
    EXPECT_EQ(count_cells_near(map, "hole", center_x, center_y, radius + 0.25f, false), 0);

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
        if (map.at("hole", *it) <= 0.5f)
        {
            continue;
        }
        // Every hole cell reaches nav2 as lethal. Two things have to be true for
        // that: it is an obstacle, and it is not "border" -- a shadowed cell with no
        // returns would otherwise publish as unknown, which the stack treats as free.
        EXPECT_GT(map.at("obstacle", *it), 0.5f);
        EXPECT_LT(map.at("border", *it), 0.5f);
        EXPECT_FLOAT_EQ(map.at("cost", *it), 100.0f);
    }
}

INSTANTIATE_TEST_SUITE_P(
    GuidebookCraters, DetectsCrater,
    ::testing::Values(
        CraterCase{"steep_50x45_at_2p5m", true, 0.50f, 0.45f, 2.5f, 0.0f},
        CraterCase{"steep_50x45_at_3m", true, 0.50f, 0.45f, 3.0f, 0.0f},
        CraterCase{"bowl_45x30_at_2m", false, 0.45f, 0.30f, 2.0f, 0.0f},
        CraterCase{"bowl_45x30_at_2m_30deg_left", false, 0.45f, 0.30f, 2.0f, 30.0f},
        CraterCase{"bowl_45x30_at_2m_45deg_right", false, 0.45f, 0.30f, 2.0f, -45.0f},
        CraterCase{"bowl_40x15_at_1p5m", false, 0.40f, 0.15f, 1.5f, 0.0f},
        CraterCase{"bowl_40x15_at_2m", false, 0.40f, 0.15f, 2.0f, 0.0f}),
    [](const ::testing::TestParamInfo<CraterCase>& crater_info)
    { return std::string(crater_info.param.name); });

TEST(HoleDetection, MarksTheShadowedFloorNotJustTheFarWall)
{
    // The cell at the middle of the crater gets no returns at all from 2.5 m: the near
    // rim hides it. It is the cell the rover would drop a wheel into, so it is the one
    // that matters, and only the ray walk can mark it.
    const auto map = classify(
        synthetic_scan::with_steep_crater(flat(), 2.5f, 0.0f, 0.5f, 0.45f));

    grid_map::Index center;
    ASSERT_TRUE(map.getIndex(grid_map::Position(2.45, 0.05), center));
    EXPECT_FLOAT_EQ(map.at("point_count", center), 0.0f);
    EXPECT_GT(map.at("hole", center), 0.5f);
}

TEST(HoleDetection, ExistingStepDownTestMissesTheSameCrater)
{
    // Why the hole test exists. Same crater, hole test off: the height-difference
    // tests flag nothing, because the slope allowance absorbs what little of the drop
    // the lidar can see.
    auto params = local_parameters();
    params.hole_depth_m = 0.0;
    const auto map = classify(
        synthetic_scan::with_bowl_crater(flat(), 2.0f, 0.0f, 0.45f, 0.30f), params);
    EXPECT_EQ(count_cells_near(map, "obstacle", 2.0f, 0.0f, 0.5f), 0);
}

// ---- The dials ---------------------------------------------------------------------

TEST(HoleDetection, ZeroDepthDisablesTheTest)
{
    auto params = local_parameters();
    params.hole_depth_m = 0.0;
    const auto map = classify(
        synthetic_scan::with_steep_crater(flat(), 2.5f, 0.0f, 0.5f, 0.45f), params);
    EXPECT_EQ(count_cells(map, "hole"), 0);
    EXPECT_TRUE(map["ground_ref"].array().isNaN().all());
}

TEST(HoleDetection, IgnoresEvidenceBeyondMaxRange)
{
    auto params = local_parameters();
    params.hole_max_range_m = 2.0;
    const auto map = classify(
        synthetic_scan::with_steep_crater(flat(), 3.0f, 0.0f, 0.5f, 0.45f), params);
    EXPECT_EQ(count_cells(map, "hole"), 0);
}

TEST(HoleDetection, UsesEachScansOwnSensorPosition)
{
    // Two scans taken half a metre apart, as on a moving rover. The rays are walked
    // back towards where each scan was taken from, so both agree on where the crater
    // is and neither smears it towards the other's position.
    const auto terrain =
        synthetic_scan::with_steep_crater(flat(), 2.5f, 0.0f, 0.5f, 0.45f);
    std::deque<local_traversability::BufferedCloud> clouds;
    clouds.push_back(synthetic_scan::scan(terrain, {}, 0.0f, 0.0f));
    clouds.push_back(synthetic_scan::scan(terrain, {}, 0.5f, 0.0f));

    grid_map::GridMap map = synthetic_scan::make_map();
    local_traversability::classify_terrain(map, clouds, local_parameters());

    EXPECT_GE(count_cells_near(map, "hole", 2.5f, 0.0f, 0.35f), 3);
    EXPECT_EQ(count_cells_near(map, "hole", 2.5f, 0.0f, 0.5f, false), 0);
}
