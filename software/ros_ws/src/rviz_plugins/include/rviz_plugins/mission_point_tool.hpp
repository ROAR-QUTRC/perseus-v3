#pragma once

/// @file mission_point_tool.hpp
/// @brief RViz tools that pick the Mission Control panel's zone waypoints on the map.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <string>

namespace rviz_plugins
{

    /// @brief Click-and-drag pose picker, exactly like RViz's own 2D Goal Pose: click
    /// sets the position, dragging sets the heading, release publishes. The pose goes
    /// out on a latched topic instead of to nav2, and MissionControlPanel picks it up,
    /// keeps it, draws its marker and sends it with the mission.
    ///
    /// One subclass per zone rather than a single tool with a "zone" property, so the
    /// panel's Pick buttons (and the toolbar) can switch straight to the right one.
    class MissionPointTool : public rviz_default_plugins::tools::PoseTool
    {
    public:
        /// @param topic Where the picked pose is published.
        /// @param name Name shown on the toolbar button.
        /// @param red,green,blue Colour of the drag arrow, matching the zone's marker.
        MissionPointTool(std::string topic, std::string name, float red, float green,
                         float blue);

        void onInitialize() override;

    protected:
        /// @brief Publishes the picked pose in RViz's fixed frame.
        void onPoseSet(double x, double y, double theta) override;

    private:
        std::string _topic;
        std::string _name;
        float _red;
        float _green;
        float _blue;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _publisher;
        rclcpp::Clock::SharedPtr _clock;
    };

    /// @brief Picks the excavation waypoint; publishes on /mission/excavation_point.
    class ExcavationPointTool : public MissionPointTool
    {
    public:
        ExcavationPointTool();
    };

    /// @brief Picks the construction waypoint; publishes on /mission/construction_point.
    class ConstructionPointTool : public MissionPointTool
    {
    public:
        ConstructionPointTool();
    };

}  // namespace rviz_plugins
