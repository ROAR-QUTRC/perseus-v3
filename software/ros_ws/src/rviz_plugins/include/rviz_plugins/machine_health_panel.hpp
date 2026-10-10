#pragma once

/// @file machine_health_panel.hpp
/// @brief RViz panel showing CPU, memory and temperature of every machine as a table.

#include <QLabel>
#include <QTableWidget>
#include <QTimer>
#include <chrono>
#include <interfaces/msg/machine_health.hpp>
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

        /// @brief Last message received from one machine, and when it arrived.
        struct MachineState
        {
            interfaces::msg::MachineHealth::ConstSharedPtr message;
            std::chrono::steady_clock::time_point received_at;
        };

        /// @brief Stores the incoming message under its topic for the next refresh.
        void _on_health(const std::string& topic,
                        interfaces::msg::MachineHealth::ConstSharedPtr message);

        /// @brief Cell color for a percentage, amber from warn and red from critical.
        static QColor _level_color(double value, double warn, double critical);

        QLabel* _summary_label{nullptr};
        QTableWidget* _table{nullptr};
        QTimer* _refresh_timer{nullptr};
        QTimer* _discovery_timer{nullptr};

        rclcpp::Node::SharedPtr _node;
        /// @brief One subscription per topic, keyed by topic name. GUI thread only.
        std::map<std::string,
                 rclcpp::Subscription<interfaces::msg::MachineHealth>::SharedPtr>
            _subscriptions;

        /// @brief Latest state per topic, shared with the executor thread.
        std::mutex _state_mutex;
        std::map<std::string, MachineState> _machines;
    };

}  // namespace rviz_plugins
