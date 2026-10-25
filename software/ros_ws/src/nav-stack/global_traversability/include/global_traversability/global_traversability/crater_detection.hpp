#pragma once

/// @file crater_detection.hpp
/// @brief Finds crater-shaped pits in the scan window's lowest-return elevation grid.

#include <grid_map_core/grid_map_core.hpp>
#include <string>
#include <vector>

namespace global_traversability
{
    /// @brief Thresholds of the crater detector.
    struct CraterParameters
    {
        /// @brief Mean slope, in degrees, from a cell out to the ring around it above which
        ///        the cell counts as a pit.
        /// @details The test is depth / ring radius, so it picks out something with
        /// walls and ignores a broad shallow dish of the same depth: the sim terrain
        /// is full of dishes 1-2 m across and ~0.1 m deep (~5 deg) that are ordinary
        /// ground, where a 0.45 m crater 0.12 m deep reads ~25 deg.
        double min_slope_deg;
        /// @brief Edges of the rings the cell is compared against, in metres, ascending.
        ///        N edges make N - 1 rings, each tried as its own scale.
        std::vector<double> ring_edges_m;
        /// @brief Fewest observed cells a ring needs to be trusted.
        int min_ring_cells;
        /// @brief Smallest observed fraction of a ring's cells it needs to be trusted.
        double min_ring_cover;
        /// @brief Radius, in metres, the flagged cells are grown by before closing.
        /// @details The test only flags a pit's steep interior -- cells whose mean slope out
        /// to the ring clears min_slope_deg -- so the wall out to the rim is left unflagged,
        /// and a 0.45 m crater comes out ~0.25 m wide. One cell (the wall's width) takes it
        /// to the rim. This restores the crater's own footprint; it is not a safety margin.
        /// Zero disables it.
        double grow_radius_m;
        /// @brief Radius, in metres, of the closing applied to the persistent crater mask
        ///        so the floor the rim hid from the lidar is covered too.
        double fill_radius_m;
        /// @brief How far, in metres, from a flagged crater cell an obstacle verdict may still
        ///        be demoted as part of the crater.
        double obstacle_margin_m;
        /// @brief Tallest, in metres above the ground, an obstacle cell near a crater may
        ///        be and still be demoted.
        /// @details The height test measures a cell against the lowest neighbour plus the
        /// slope allowance, so the rim cell one or two cells from a deep floor reads as
        /// something standing on the ground -- lethal, with its own inflation halo. Those
        /// cells stand ~0.08 m. This cap is what keeps the demotion from ever touching a
        /// wall (0.85 m) or a rock: anything taller than this stays an obstacle however
        /// close to a pit it is.
        double demote_max_height_m;
    };

    /// @brief Fills the "crater" layer of @p window from its "elevation", "border" and
    ///        "obstacle" layers: 1 for a pit, 0 for evaluated ground, NaN for no verdict.
    /// @details THE TEST. A cell is a pit when the median elevation of the cells in a
    /// ring around it, minus its own, exceeds ring radius * tan(min_slope_deg). The
    /// median cancels a planar slope and shrugs off a rock or two in the ring; the
    /// division by radius is what separates a crater from a dish. It is tried at each
    /// ring in params.ring_edges_m and the steepest wins, so one setting covers craters
    /// of several sizes.
    ///
    /// NO VERDICT WITHOUT SUPPORT. The lidar's ground returns lie on thin concentric arcs
    /// (it is pitched down, 32 rings), so a cell's ring is often all on one side or a
    /// line. A ring is used only when enough of it was observed AND the observed cells
    /// include two on roughly opposite sides of the cell: the cell is interpolated
    /// between its surroundings, never extrapolated from one side of a slope or an edge.
    /// A cell with no usable ring is left NaN, which the node reads as "no evidence".
    ///
    /// GROUND ONLY. Cells the height test already calls obstacles are neither pits nor
    /// part of a ring: a cell at a wall's foot would otherwise have its ring median
    /// dragged up by the wall and read as a pit (on the real rock_nav_test_2 bag, 263
    /// cells along the arena wall did exactly that).
    ///
    /// Reads the per-cell LOWEST return rather than a mean: with 2 cm of range noise the
    /// lowest return scores almost exactly as it does noiseless, where a mean smears the
    /// crater wall across the cells it straddles and loses about half the detections.
    /// @param window Scan window carrying "elevation", "border", "obstacle" and "crater".
    /// @param params Detector thresholds.
    void detect_craters(grid_map::GridMap& window, const CraterParameters& params);

    /// @brief Clears the "obstacle" verdict of low cells beside a flagged crater cell.
    /// @details Run on the window after detect_craters(), so what gets fused is already
    /// demoted and the persistent obstacle log-odds just see the cell as free. A cell is
    /// demoted only if it is within params.obstacle_margin_m of a "crater" cell AND its
    /// "height_above_ground" is at most params.demote_max_height_m.
    /// @param window Scan window carrying "obstacle", "height_above_ground" and "crater".
    /// @param params Margin and height cap.
    void demote_crater_obstacles(grid_map::GridMap& window, const CraterParameters& params);

    /// @brief Grows the set cells of a 0/1 layer by @p radius_m, in place.
    /// @param map Grid carrying @p layer.
    /// @param layer Layer to dilate; a cell is set when its value exceeds 0.5.
    /// @param radius_m Dilation radius.
    void dilate_mask(grid_map::GridMap& map, const std::string& layer, double radius_m);

    /// @brief Morphological closing of a 0/1 layer: fills gaps narrower than twice
    ///        @p radius_m without growing the outline.
    /// @param map Grid carrying @p layer.
    /// @param layer Layer to close in place; a cell is set when its value exceeds 0.5.
    /// @param radius_m Closing radius.
    void close_mask(grid_map::GridMap& map, const std::string& layer, double radius_m);
}  // namespace global_traversability
