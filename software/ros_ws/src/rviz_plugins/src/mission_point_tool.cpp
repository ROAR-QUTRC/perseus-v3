/// @file mission_point_tool.cpp
/// @brief Implementation of MissionPointTool and its two zone subclasses.

#include "rviz_plugins/mission_point_tool.hpp"

#include <tf2/exceptions.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

namespace rviz_plugins
{

    namespace
    {
        // arena_server's frame: the guidebook arena, re-fitted by every fiducial fix.
        constexpr const char* MAP_FRAME = "map";
    }  // namespace

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
        _tf_buffer = std::make_shared<tf2_ros::Buffer>(_clock);
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
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

        // RViz picks in its fixed frame, odom, which is pinned to where the rover
        // powered on. The arena is not: a fiducial fix (the panel's Reposition button,
        // or the mission's scan) moves map -> odom. A point kept in odom would stay put
        // while the arena slid away under it, so keep it in map, where it moves with
        // the arena and is resolved to odom only when nav2 drives to it.
        if (pose.header.frame_id != MAP_FRAME)
        {
            try
            {
                const auto to_map = _tf_buffer->lookupTransform(
                    MAP_FRAME, pose.header.frame_id, tf2::TimePointZero);
                geometry_msgs::msg::PoseStamped in_map;
                tf2::doTransform(pose, in_map, to_map);
                in_map.header.frame_id = MAP_FRAME;
                in_map.header.stamp = pose.header.stamp;
                pose = in_map;
            }
            catch (const tf2::TransformException& e)
            {
                RCLCPP_WARN(rclcpp::get_logger("mission_point_tool"),
                            "%s: no %s -> %s transform (%s); keeping the point in %s, "
                            "which will not follow a re-localisation",
                            _name.c_str(), MAP_FRAME, pose.header.frame_id.c_str(), e.what(),
                            pose.header.frame_id.c_str());
            }
        }
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
