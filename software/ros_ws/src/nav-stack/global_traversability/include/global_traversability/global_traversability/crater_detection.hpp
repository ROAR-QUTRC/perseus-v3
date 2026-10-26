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
        /// @brief Smallest fraction of a ring's observed sectors that must sit clearly above
        ///        the cell for it to count as a pit.
        /// @details A crater is higher on every side; the foot of a bank or wall is higher
        /// on one and level on the others, yet scores the same on the ring median because
        /// the uphill half drags it up. Each of the ring's 8 sectors that was observed must
        /// rise above the cell by half of what the slope test needs (tan(min_slope_deg) *
        /// ring radius / 2); this is the fraction of them that must.
        double min_rim_fraction;
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
        /// @brief Height above the ground, in metres, beyond which an obstacle cell is a
        ///        structure: a wall or a rock, never a pit and never part of one.
        /// @details Structure cells are left out of the crater test (as the cell or as part
        /// of its ring) and are never demoted. Shorter obstacle cells stay eligible: the
        /// height test also flags a crater's own rim and, under noise, bits of its floor.
        double structure_height_m;
    };

    /// @brief Fills the "crater" and "crater_depth" layers of @p window from its
    ///        "elevation", "border", "obstacle" and "height_above_ground" layers.
    /// @details "crater" is 1 for a pit, 0 for evaluated ground, NaN for no verdict.
    /// "crater_depth" is how far, in metres, the cell sits below the median of the ring
    /// that scored it steepest.
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
    /// SURROUNDED, NOT JUST BRACKETED. A ring also has to rise on (almost) every side it was
    /// seen from, see CraterParameters::min_rim_fraction: the concave corner at the foot of
    /// a bank passes the median test and fails this one.
    ///
    /// GROUND ONLY. Structures (obstacle cells taller than params.structure_height_m) are
    /// neither pits nor part of a ring: a cell at a wall's foot would otherwise have its
    /// ring median dragged up by the wall and read as a pit (on the real rock_nav_test_2
    /// bag, 263 cells along the arena wall did exactly that).
    ///
    /// Reads the per-cell LOWEST return rather than a mean: with 2 cm of range noise the
    /// lowest return scores almost exactly as it does noiseless, where a mean smears the
    /// crater wall across the cells it straddles and loses about half the detections.
    /// @param window Scan window carrying the layers above plus "crater", "crater_depth".
    /// @param params Detector thresholds.
    void detect_craters(grid_map::GridMap& window, const CraterParameters& params);

    /// @brief Clears the "obstacle" verdict of cells that are only a crater's rim.
    /// @details Run on the window after detect_craters(), so what gets fused is already
    /// demoted and the persistent obstacle log-odds just see the cell as free. The height
    /// test measures a cell against the lowest neighbour plus the slope allowance, so the
    /// rim cell next to a deep floor reads as standing on the ground -- but it can never
    /// read taller than the pit is deep, because the pit is the only reason the ground
    /// estimate is low. So a cell within params.obstacle_margin_m of a flagged cell is
    /// demoted when its "height_above_ground" is no more than that cell's "crater_depth"
    /// (plus a little for noise) and under params.structure_height_m. A wall, 0.85 m
    /// beside even a 0.25 m crater, is neither.
    /// @param window Scan window carrying "obstacle", "height_above_ground", "crater" and
    ///        "crater_depth".
    /// @param params Margin and structure height.
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
