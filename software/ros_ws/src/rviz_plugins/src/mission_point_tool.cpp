/// @file mission_point_tool.cpp
/// @brief Implementation of MissionPointTool and its two zone subclasses.

#include "rviz_plugins/mission_point_tool.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <utility>

namespace rviz_plugins
{

    MissionPointTool::MissionPointTool(std::string topic, std::string name, float red,
                                       float green, float blue)
        : _topic(std::move(topic)),
          _name(std::move(name)),
          _red(red),
          _green(green),
          _blue(blue)
    {
    }

    void MissionPointTool::onInitialize()
    {
        PoseTool::onInitialize();
        arrow_->setColor(_red, _green, _blue, 1.0f);
        setName(QString::fromStdString(_name));

        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
        // Latched: the panel keeps the pose itself, but a panel reopened after the
        // pick (or a second base station) still gets the latest one.
        _publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
            _topic, rclcpp::QoS(1).reliable().transient_local());
        _clock = node->get_clock();
    }

    void MissionPointTool::onPoseSet(double x, double y, double theta)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = context_->getFixedFrame().toStdString();
        pose.header.stamp = _clock->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation = orientationAroundZAxis(theta);
        logPose(_name, pose.pose.position, pose.pose.orientation, theta,
                pose.header.frame_id);
        _publisher->publish(pose);
    }

    // Colours match MissionControlPanel's markers and zone cards.
    ExcavationPointTool::ExcavationPointTool()
        : MissionPointTool("/mission/excavation_point", "Excavation Point", 0.96f, 0.65f,
                           0.14f)
    {
    }

    ConstructionPointTool::ConstructionPointTool()
        : MissionPointTool("/mission/construction_point", "Construction Point", 0.18f,
                           0.77f, 0.71f)
    {
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::ExcavationPointTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ConstructionPointTool, rviz_common::Tool)
