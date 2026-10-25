#pragma once

/// @file bucket_trees.hpp
/// @brief Loading tree files saved by Groot2 - mission.xml, and the bucket behaviours built
/// from saved poses (autonomy_bringup's behavior_trees/bucket/) - shared by
/// mission_bt_server and bucket_cli so the mission and the command line register the same
/// trees the same way.

#include <behaviortree_cpp/bt_factory.h>

#include <filesystem>
#include <vector>

namespace mission_bt_server
{

    /// @brief Every *.xml in @p dir, sorted, so trees register in a stable order.
    std::vector<std::filesystem::path> bucket_tree_files(const std::filesystem::path& dir);

    /// @brief Registers the trees in @p file with its blank ports dropped. Groot2 saves
    /// every port of a node, a blank one as name="", and BT.CPP reads that as the literal
    /// "" (which no number converts from) instead of unset. Dropped, a blank port takes
    /// its default, or is omitted where it has none: a joint holds, a move is direct.
    /// Every file Groot2 has saved needs this, mission.xml included. Throws with the
    /// file's name on a tree that will not load.
    void register_tree_file(BT::BehaviorTreeFactory& factory,
                            const std::filesystem::path& file);

    /// @brief register_tree_file for every bucket_tree_files(@p dir).
    void register_bucket_trees(BT::BehaviorTreeFactory& factory,
                               const std::filesystem::path& dir);

}  // namespace mission_bt_server
