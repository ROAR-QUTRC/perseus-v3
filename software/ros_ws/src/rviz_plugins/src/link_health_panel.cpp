/// @file link_health_panel.cpp
/// @brief Implementation of the link health RViz panel.

#include "rviz_plugins/link_health_panel.hpp"

#include <QHBoxLayout>
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_plugins
{
    namespace
    {

        // The same palette as the other health panels, so they read alike.
        const QColor COLOUR_OK(90, 190, 110);
        const QColor COLOUR_DEGRADED(240, 170, 60);
        const QColor COLOUR_ERROR(225, 90, 90);
        const QColor COLOUR_NO_DATA(170, 170, 170);

        constexpr double BITS_PER_BYTE = 8.0;
        constexpr double BITS_PER_MEGABIT = 1.0e6;

        QColor link_colour(std::uint8_t status)
        {
            switch (status)
            {
            case interfaces::msg::LinkHealth::STATUS_OK:
                return COLOUR_OK;
            case interfaces::msg::LinkHealth::STATUS_DEGRADED:
                return COLOUR_DEGRADED;
            default:
                return COLOUR_ERROR;
            }
        }

        QColor device_colour(std::uint8_t status)
        {
            switch (status)
            {
            case interfaces::msg::DeviceLink::STATUS_OK:
                return COLOUR_OK;
            case interfaces::msg::DeviceLink::STATUS_DEGRADED:
                return COLOUR_DEGRADED;
            default:
                return COLOUR_ERROR;
            }
        }

    }  // namespace

    LinkHealthPanel::LinkHealthPanel(QWidget* parent)
        : rviz_common::Panel(parent)
    {
        _layout = new QVBoxLayout;
        setLayout(_layout);

        _link_row = _make_row();
        _link_row.name->setText("Rover link");
        _link_row.value->setText("no data");

        _throughput_bar = new QProgressBar;
        _throughput_bar->setTextVisible(false);
        _throughput_bar->setRange(0, 1000);
        _throughput_bar->setFixedHeight(8);
        _layout->addWidget(_throughput_bar);
        _layout->addSpacing(8);
        _layout->addStretch();

        _set_dot(_link_row, COLOUR_NO_DATA);

        _refresh_timer = new QTimer(this);
        connect(_refresh_timer, &QTimer::timeout, this, &LinkHealthPanel::_refresh);
    }

    LinkHealthPanel::Row LinkHealthPanel::_make_row()
    {
        Row row;
        row.dot = new QLabel(QString::fromUtf8("●"));
        row.name = new QLabel;
        row.value = new QLabel;
        row.value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        auto* line = new QHBoxLayout;
        line->addWidget(row.dot);
        line->addWidget(row.name, 1);
        line->addWidget(row.value);

        // Before the trailing stretch, so rows stay packed at the top.
        _layout->insertLayout(std::max(0, _layout->count() - 1), line);
        return row;
    }

    void LinkHealthPanel::_set_dot(const Row& row, const QColor& colour)
    {
        row.dot->setStyleSheet(
            QString("color: %1; font-size: 16px;").arg(colour.name()));
    }

    void LinkHealthPanel::_resize_device_rows(std::size_t count)
    {
        while (_device_rows.size() < count)
        {
            _device_rows.push_back(_make_row());
        }
        while (_device_rows.size() > count)
        {
            const Row& row = _device_rows.back();
            // Removing the row's widgets leaves its empty layout behind, which takes no space.
            delete row.dot;
            delete row.name;
            delete row.value;
            _device_rows.pop_back();
        }
    }

    void LinkHealthPanel::onInitialize()
    {
        _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        _subscription = _node->create_subscription<interfaces::msg::LinkHealth>(
            TOPIC, rclcpp::QoS(1),
            [this](interfaces::msg::LinkHealth::ConstSharedPtr message)
            {
                const std::lock_guard<std::mutex> lock(_state_mutex);
                _latest = std::move(message);
                _received_at = std::chrono::steady_clock::now();
            });
        _refresh_timer->start(REFRESH_PERIOD_MS);
    }

    void LinkHealthPanel::_refresh()
    {
        interfaces::msg::LinkHealth::ConstSharedPtr message;
        std::chrono::steady_clock::time_point received_at;
        {
            const std::lock_guard<std::mutex> lock(_state_mutex);
            message = _latest;
            received_at = _received_at;
        }

        const bool stale =
            !message || std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                      received_at)
                                .count() > STALE_AFTER_SEC;
        if (stale)
        {
            // Grey rather than the last colour: the monitor that would turn it red is the
            // thing that has gone quiet.
            _set_dot(_link_row, COLOUR_NO_DATA);
            _link_row.value->setText("no data");
            _link_row.dot->setToolTip(QString("Nothing received on %1").arg(TOPIC));
            _throughput_bar->setValue(0);
            for (const auto& row : _device_rows)
            {
                _set_dot(row, COLOUR_NO_DATA);
                row.value->setText("");
            }
            return;
        }

        // Link row: the rate coming from the rover, which is the direction that matters.
        _set_dot(_link_row, link_colour(message->overall_status));
        const bool have_rate = message->rx_bytes_per_sec >= 0.0;
        const double rx_mbps =
            have_rate ? message->rx_bytes_per_sec * BITS_PER_BYTE / BITS_PER_MEGABIT : 0.0;
        const double tx_mbps =
            have_rate ? message->tx_bytes_per_sec * BITS_PER_BYTE / BITS_PER_MEGABIT : 0.0;
        _link_row.value->setText(
            message->interface.empty() ? QString("no route")
            : have_rate                ? QString("%1 Mbps").arg(rx_mbps, 0, 'f', 1)
                                       : QString("..."));

        // Full scale is the negotiated speed when there is one; otherwise the busiest the
        // link has been, so the bar still shows how loaded it is relative to what it has done.
        double scale_mbps = message->link_speed_mbps;
        if (scale_mbps <= 0.0)
        {
            _peak_mbps = std::max({_peak_mbps, rx_mbps, tx_mbps});
            scale_mbps = _peak_mbps;
        }
        _throughput_bar->setValue(static_cast<int>(
            std::clamp(rx_mbps / scale_mbps, 0.0, 1.0) * _throughput_bar->maximum()));
        _throughput_bar->setStyleSheet(
            QString("QProgressBar::chunk { background-color: %1; }")
                .arg(link_colour(message->overall_status).name()));

        _link_row.dot->setToolTip(
            QString("Interface: %1\nFrom rover: %2 Mbps\nTo rover: %3 Mbps\nLink speed: %4")
                .arg(message->interface.empty() ? QString("none")
                                                : QString::fromStdString(message->interface))
                .arg(rx_mbps, 0, 'f', 2)
                .arg(tx_mbps, 0, 'f', 2)
                .arg(message->link_speed_mbps > 0.0F
                         ? QString("%1 Mbps").arg(message->link_speed_mbps, 0, 'f', 0)
                         : QString("unknown")));
        _throughput_bar->setToolTip(_link_row.dot->toolTip());

        // Device rows: dot and round trip, or DOWN.
        _resize_device_rows(message->devices.size());
        for (std::size_t i = 0; i < message->devices.size(); ++i)
        {
            const auto& device = message->devices[i];
            const auto& row = _device_rows[i];
            _set_dot(row, device_colour(device.status));
            row.name->setText(QString::fromStdString(device.name));
            row.value->setText(device.status == interfaces::msg::DeviceLink::STATUS_DOWN
                                   ? QString("DOWN")
                                   : QString("%1 ms").arg(device.rtt_ms, 0, 'f', 0));

            const QString detail =
                QString("%1\nLoss: %2%\nJitter: %3")
                    .arg(QString::fromStdString(device.ip))
                    .arg(device.loss_percent, 0, 'f', 0)
                    .arg(device.jitter_ms >= 0.0F
                             ? QString("%1 ms").arg(device.jitter_ms, 0, 'f', 1)
                             : QString("n/a"));
            row.dot->setToolTip(detail);
            row.name->setToolTip(detail);
            row.value->setToolTip(detail);
        }
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::LinkHealthPanel, rviz_common::Panel)
