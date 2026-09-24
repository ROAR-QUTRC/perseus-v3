/// @file mission_control_panel.cpp
/// @brief Implementation of MissionControlPanel.

#include "rviz_plugins/mission_control_panel.hpp"

#include <QVBoxLayout>
#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_plugins
{

    using namespace std::chrono_literals;

    MissionControlPanel::MissionControlPanel(QWidget* parent)
        : rviz_common::Panel(parent)
    {
        _excavation_button = new QPushButton("Move to Excavation Zone");
        _construction_button = new QPushButton("Move to Construction Zone");
        _status_label = new QLabel("Idle");
        _status_label->setAlignment(Qt::AlignCenter);
        _status_label->setWordWrap(true);

        auto* layout = new QVBoxLayout();
        layout->addWidget(_excavation_button);
        layout->addWidget(_construction_button);
        layout->addWidget(_status_label);
        setLayout(layout);

        connect(_excavation_button, &QPushButton::clicked, this,
                &MissionControlPanel::_on_excavation_clicked);
        connect(_construction_button, &QPushButton::clicked, this,
                &MissionControlPanel::_on_construction_clicked);
    }

    void MissionControlPanel::onInitialize()
    {
        _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        _excavation_client = _node->create_client<std_srvs::srv::Trigger>(
            _excavation_service.toStdString());
        _construction_client = _node->create_client<std_srvs::srv::Trigger>(
            _construction_service.toStdString());

        _poll_timer = new QTimer(this);
        connect(_poll_timer, &QTimer::timeout, this, &MissionControlPanel::_poll);
        _poll_timer->start(POLL_PERIOD_MS);
    }

    void MissionControlPanel::_on_excavation_clicked()
    {
        _send_request(_excavation_client, _excavation_service, "excavation zone");
    }

    void MissionControlPanel::_on_construction_clicked()
    {
        _send_request(_construction_client, _construction_service,
                      "construction zone");
    }

    void MissionControlPanel::_send_request(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
        const QString& service_name, const QString& zone_label)
    {
        if (_pending_future)
        {
            _status_label->setText("Still navigating to " + _pending_zone_label +
                                   " - ignored");
            return;
        }
        if (!client->service_is_ready())
        {
            _status_label->setText(service_name +
                                   " is not available (mission_bt_server not up?)");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        _pending_future = client->async_send_request(request);
        _pending_zone_label = zone_label;
        _excavation_button->setEnabled(false);
        _construction_button->setEnabled(false);
        _status_label->setStyleSheet("");
        _status_label->setText("Navigating to " + zone_label + "...");
    }

    void MissionControlPanel::_poll()
    {
        if (!_pending_future)
            return;
        if (_pending_future->wait_for(0s) != std::future_status::ready)
            return;

        const auto response = _pending_future->get();
        _pending_future.reset();
        _excavation_button->setEnabled(true);
        _construction_button->setEnabled(true);

        if (response->success)
        {
            _status_label->setStyleSheet("color: #2e8b2e;");
            _status_label->setText("Arrived at " + _pending_zone_label);
        }
        else
        {
            _status_label->setStyleSheet("color: #b32424;");
            _status_label->setText("Failed to reach " + _pending_zone_label + ": " +
                                   QString::fromStdString(response->message));
        }
    }

    void MissionControlPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("ExcavationService", _excavation_service);
        config.mapSetValue("ConstructionService", _construction_service);
    }

    void MissionControlPanel::load(const rviz_common::Config& config)
    {
        rviz_common::Panel::load(config);
        config.mapGetString("ExcavationService", &_excavation_service);
        config.mapGetString("ConstructionService", &_construction_service);
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::MissionControlPanel, rviz_common::Panel)
