/// @file bucket_trees.cpp
/// @brief Bucket behaviour loading shared by mission_bt_server and bucket_cli.

#include "mission_bt_server/mission_bt_server/bucket_trees.hpp"

#include <algorithm>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace mission_bt_server
{

    namespace fs = std::filesystem;

    std::vector<fs::path> bucket_tree_files(const fs::path& dir)
    {
        std::vector<fs::path> files;
        for (const auto& entry : fs::directory_iterator(dir))
            if (entry.is_regular_file() && entry.path().extension() == ".xml")
                files.push_back(entry.path());
        std::sort(files.begin(), files.end());
        return files;
    }

    void register_bucket_tree_file(BT::BehaviorTreeFactory& factory, const fs::path& file)
    {
        static const std::regex BLANK_PORT(R"(\s+[A-Za-z_][A-Za-z0-9_]*="")");
        std::ifstream in(file);
        std::stringstream text;
        text << in.rdbuf();
        try
        {
            factory.registerBehaviorTreeFromText(
                std::regex_replace(text.str(), BLANK_PORT, ""));
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error(file.filename().string() + ": " + e.what());
        }
    }

    void register_bucket_trees(BT::BehaviorTreeFactory& factory, const fs::path& dir)
    {
        for (const auto& file : bucket_tree_files(dir))
            register_bucket_tree_file(factory, file);
    }

}  // namespace mission_bt_server
