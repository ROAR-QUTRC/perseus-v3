/// @file wall_side_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for WallSideBtNode.

#include "mission_bt_server/mission_bt_server/wall_side_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <limits>
#include <string>

namespace mission_bt_server
{

    namespace
    {
        // How long to wait for a costmap and an odometry message from after the tick.
        constexpr auto DATA_WAIT = std::chrono::seconds(3);
        constexpr auto SPIN_BUDGET = std::chrono::milliseconds(20);

        double yaw_of(const geometry_msgs::msg::Quaternion& q)
        {
            return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }
    }  // namespace

    WallSideBtNode::WallSideBtNode(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::StatefulActionNode(name, conf)
    {
        _node = config().blackboard->get<rclcpp::Node::SharedPtr>("@node");
        _callback_group =
            _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        _executor.add_callback_group(_callback_group, _node->get_node_base_interface());

        rclcpp::SubscriptionOptions options;
        options.callback_group = _callback_group;
        // Matches local_traversability's publisher: reliable, volatile, depth 1.
        _grid_subscription = _node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap_terrain", rclcpp::QoS(1).reliable(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
            { _grid = std::move(msg); },
            options);
        // Best effort connects to either kind of publisher.
        _odom_subscription = _node->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            { _odom = std::move(msg); },
            options);
    }

    BT::PortsList WallSideBtNode::providedPorts()
    {
        return {
            BT::InputPort<double>("step_deg", 30.0, "size of the turn written to spin_dist"),
            BT::InputPort<double>("range_m", 4.0, "ignore walls farther than this"),
            BT::InputPort<int>("min_cost", 99,
                               "costmap value from which a cell counts as a wall"),
            BT::OutputPort<double>("spin_dist",
                                   "step_deg in radians: positive left, negative right"),
            BT::OutputPort<std::string>("side", "left or right"),
        };
    }

    BT::NodeStatus WallSideBtNode::onStart()
    {
        double step_deg = 30.0;
        getInput("step_deg", step_deg);
        getInput("range_m", _range_m);
        getInput("min_cost", _min_cost);
        _step_rad = std::abs(step_deg) * M_PI / 180.0;

        // Only data from after now counts: the rover may have been turned since the
        // last grid, and the first one in hand is not the one to trust.
        _grid.reset();
        _odom.reset();
        _deadline = std::chrono::steady_clock::now() + DATA_WAIT;
        return onRunning();
    }

    BT::NodeStatus WallSideBtNode::onRunning()
    {
        _executor.spin_some(SPIN_BUDGET);
        if (_grid && _odom)
        {
            if (const auto left = _nearest_side())
            {
                return _finish(*left, "nearest wall");
            }
            return _finish(true, "no wall in range, defaulting");
        }
        if (std::chrono::steady_clock::now() >= _deadline)
        {
            return _finish(true, "no costmap or odometry, defaulting");
        }
        return BT::NodeStatus::RUNNING;
    }

    void WallSideBtNode::onHalted()
    {
    }

    std::optional<bool> WallSideBtNode::_nearest_side() const
    {
        const auto& grid = *_grid;
        const double yaw = yaw_of(_odom->pose.pose.orientation);
        const double rover_x = _odom->pose.pose.position.x;
        const double rover_y = _odom->pose.pose.position.y;
        // Heading axes: x forward, y to the left.
        const double fx = std::cos(yaw), fy = std::sin(yaw);

        const double origin_yaw = yaw_of(grid.info.origin.orientation);
        const double oc = std::cos(origin_yaw), os = std::sin(origin_yaw);
        const double res = grid.info.resolution;

        const double inf = std::numeric_limits<double>::infinity();
        double nearest_left = inf, nearest_right = inf;
        for (uint32_t row = 0; row < grid.info.height; ++row)
        {
            for (uint32_t col = 0; col < grid.info.width; ++col)
            {
                if (grid.data[row * grid.info.width + col] < _min_cost)
                {
                    continue;
                }
                const double cx = (col + 0.5) * res, cy = (row + 0.5) * res;
                const double wx = grid.info.origin.position.x + oc * cx - os * cy;
                const double wy = grid.info.origin.position.y + os * cx + oc * cy;
                const double dx = wx - rover_x, dy = wy - rover_y;
                const double distance = std::hypot(dx, dy);
                if (distance > _range_m)
                {
                    continue;
                }
                // z of heading x offset: positive when the cell is to the left.
                const bool left = fx * dy - fy * dx > 0.0;
                double& nearest = left ? nearest_left : nearest_right;
                nearest = std::min(nearest, distance);
            }
        }
        if (nearest_left == inf && nearest_right == inf)
        {
            return std::nullopt;
        }
        RCLCPP_INFO(_node->get_logger(), "WallSide: nearest wall %.2f m left, %.2f m right",
                    nearest_left, nearest_right);
        return nearest_left <= nearest_right;
    }

    BT::NodeStatus WallSideBtNode::_finish(bool left, const char* why)
    {
        setOutput("spin_dist", left ? _step_rad : -_step_rad);
        setOutput("side", std::string(left ? "left" : "right"));
        RCLCPP_INFO(_node->get_logger(), "WallSide: spinning %s (%s)", left ? "left" : "right",
                    why);
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace mission_bt_server

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mission_bt_server::WallSideBtNode>("WallSide");
}
