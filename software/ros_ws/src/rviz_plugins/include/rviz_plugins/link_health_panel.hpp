#pragma once

/// @file link_health_panel.hpp
/// @brief RViz panel showing at a glance whether the radio link to the rover is good.

#include <QLabel>
#include <QProgressBar>
#include <QTimer>
#include <QVBoxLayout>
#include <chrono>
#include <interfaces/msg/link_health.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <vector>

namespace rviz_plugins
{

    /// @brief Dockable RViz panel with a status dot for the link and one for each rover
    /// device, fed by link_monitor's LinkHealth.
    ///
    /// It deliberately shows little: a colour per row, the round trip, and one throughput
    /// bar. Detail (loss, jitter, interface, upload rate) is in the tooltips. The colours
    /// come straight from the status link_monitor decided, so thresholds live in one
    /// place. When messages stop arriving the whole panel goes grey, because a monitor that
    /// has died must not leave the last green dot on screen.
    ///
    /// As in the other panels, the subscription callback runs on an executor thread and
    /// only stores the latest message; a GUI-thread timer repaints from it.
    class LinkHealthPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        /// @brief Builds the widgets. Subscribing waits for onInitialize().
        /// @param parent Parent widget, owned by RViz.
        explicit LinkHealthPanel(QWidget* parent = nullptr);

        /// @brief Gets the RViz node, subscribes and starts the repaint timer.
        void onInitialize() override;

    private Q_SLOTS:
        /// @brief Repaints from the latest message.
        void _refresh();

    private:
        /// @brief Topic link_monitor publishes on.
        static inline const char* TOPIC = "/link_health";
        /// @brief Repaint period, in milliseconds.
        static constexpr int REFRESH_PERIOD_MS = 200;
        /// @brief Seconds without a message before the panel is greyed out.
        static constexpr double STALE_AFTER_SEC = 3.0;
        /// @brief Smallest full-scale of the throughput bar when the link speed is unknown,
        /// in megabits per second.
        static constexpr double MIN_BAR_SCALE_MBPS = 10.0;

        /// @brief One row: a coloured dot, a name and a value.
        struct Row
        {
            QLabel* dot{nullptr};
            QLabel* name{nullptr};
            QLabel* value{nullptr};
        };

        /// @brief Creates a row in the layout, to be filled by _refresh().
        Row _make_row();

        /// @brief Sets a row's dot colour from a status, or grey for no data.
        static void _set_dot(const Row& row, const QColor& colour);

        /// @brief Makes the device rows match the message, adding or removing as needed.
        void _resize_device_rows(std::size_t count);

        QVBoxLayout* _layout{nullptr};
        Row _link_row;
        QProgressBar* _throughput_bar{nullptr};
        std::vector<Row> _device_rows;
        QTimer* _refresh_timer{nullptr};

        rclcpp::Node::SharedPtr _node;
        rclcpp::Subscription<interfaces::msg::LinkHealth>::SharedPtr _subscription;

        /// @brief Latest message, shared with the executor thread.
        std::mutex _state_mutex;
        interfaces::msg::LinkHealth::ConstSharedPtr _latest;
        std::chrono::steady_clock::time_point _received_at;

        /// @brief Largest throughput seen, so the bar has a scale on a link whose speed the
        /// kernel does not report (wifi). GUI thread only.
        double _peak_mbps{MIN_BAR_SCALE_MBPS};
    };

}  // namespace rviz_plugins
