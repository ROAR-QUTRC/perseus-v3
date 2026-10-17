#pragma once

/// @file machine_health_panel.hpp
/// @brief RViz panel showing CPU, memory and temperature of every machine as a table,
/// and the rover control board's power buses as a second one.

#include <QLabel>
#include <QTableWidget>
#include <QTimer>
#include <chrono>
#include <interfaces/msg/machine_health.hpp>
#include <interfaces/msg/rcb_power_status.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <string>

namespace rviz_plugins
{

    /// @brief Dockable RViz panel with one row per machine publishing MachineHealth.
    ///
    /// Machines are found rather than configured: every DISCOVERY_PERIOD_MS the panel
    /// lists the topics on the graph and subscribes to each /<device_name>/system_health
    /// it has not seen before, so a machine that starts late simply appears. A row stays
    /// in the table once seen and is shown as stale when its messages stop, because a
    /// machine that went quiet is exactly the thing the operator needs to notice.
    ///
    /// As in TopicHealthPanel, subscription callbacks run on an executor thread and only
    /// store the latest message per machine; a GUI-thread timer repaints from that.
    ///
    /// The second table, beside the first, is the rover control board's power buses (compute, drive, aux,
    /// spare), from RcbPowerStatus on /can_to_ros, which rcb_driver publishes once per
    /// bus. Unlike the machines it is one known topic, so it is subscribed to straight
    /// away, and a bus is shown stale like a machine when its messages stop.
    class MachineHealthPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        /// @brief Builds the widgets. Subscribing waits for onInitialize().
        /// @param parent Parent widget, owned by RViz.
        explicit MachineHealthPanel(QWidget* parent = nullptr);

        /// @brief Gets the RViz node and starts the discovery and refresh timers.
        void onInitialize() override;

    private Q_SLOTS:
        /// @brief Subscribes to any new /<device_name>/system_health topic.
        void _discover();

        /// @brief Repaints the table from the latest message of each machine.
        void _refresh();

    private:
        /// @brief Suffix that marks a machine's topic.
        static inline const std::string TOPIC_SUFFIX = "/system_health";
        /// @brief Message type the topic must carry.
        static inline const std::string TOPIC_TYPE = "interfaces/msg/MachineHealth";
        /// @brief Queue depth of each subscription.
        static constexpr int SUBSCRIPTION_QUEUE_DEPTH = 10;
        /// @brief Repaint period, in milliseconds.
        static constexpr int REFRESH_PERIOD_MS = 200;
        /// @brief Graph scan period, in milliseconds.
        static constexpr int DISCOVERY_PERIOD_MS = 2000;
        /// @brief Seconds without a message before a machine is shown as stale.
        ///
        /// Three missed publishes at the monitor's default 1 Hz.
        static constexpr double STALE_AFTER_SEC = 3.0;
        /// @brief Topic rcb_driver publishes each bus's status on.
        static inline const std::string RCB_TOPIC = "/can_to_ros";
        /// @brief Seconds without a status from a bus before it is shown as stale.
        static constexpr double RCB_STALE_AFTER_SEC = 5.0;

        /// @brief Last message received from one machine, and when it arrived.
        struct MachineState
        {
            interfaces::msg::MachineHealth::ConstSharedPtr message;
            std::chrono::steady_clock::time_point received_at;
        };

        /// @brief Last status received for one power bus, and when it arrived.
        struct BusState
        {
            interfaces::msg::RcbPowerStatus::ConstSharedPtr message;
            std::chrono::steady_clock::time_point received_at;
        };

        /// @brief Repaints the power bus table from the latest status of each bus.
        void _refresh_buses();

        /// @brief Stores the incoming bus status under its bus name for the next refresh.
        void _on_bus_status(interfaces::msg::RcbPowerStatus::ConstSharedPtr message);

        /// @brief Stores the incoming message under its topic for the next refresh.
        void _on_health(const std::string& topic,
                        interfaces::msg::MachineHealth::ConstSharedPtr message);

        /// @brief Cell color for a percentage, amber from warn and red from critical.
        static QColor _level_color(double value, double warn, double critical);

        QLabel* _summary_label{nullptr};
        QTableWidget* _table{nullptr};
        QLabel* _bus_label{nullptr};
        QTableWidget* _bus_table{nullptr};
        QTimer* _refresh_timer{nullptr};
        QTimer* _discovery_timer{nullptr};

        rclcpp::Node::SharedPtr _node;
        /// @brief One subscription per topic, keyed by topic name. GUI thread only.
        std::map<std::string,
                 rclcpp::Subscription<interfaces::msg::MachineHealth>::SharedPtr>
            _subscriptions;

        rclcpp::Subscription<interfaces::msg::RcbPowerStatus>::SharedPtr _bus_subscription;
        /// @brief Latest status per bus name, shared with the executor thread.
        std::mutex _bus_mutex;
        std::map<std::string, BusState> _buses;

        /// @brief Latest state per topic, shared with the executor thread.
        std::mutex _state_mutex;
        std::map<std::string, MachineState> _machines;
    };

}  // namespace rviz_plugins
