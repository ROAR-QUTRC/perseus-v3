/// @file mission_control_panel.cpp
/// @brief Implementation of MissionControlPanel.

#include "rviz_plugins/mission_control_panel.hpp"

#include <QFormLayout>
#include <QGraphicsBlurEffect>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>

namespace rviz_plugins
{

    using namespace std::chrono_literals;
    using StartMission = interfaces::srv::StartMission;
    using MissionStatus = interfaces::msg::MissionStatus;

    namespace
    {
        const QString EXCAVATION_TOOL = "rviz_plugins/ExcavationPoint";
        const QString CONSTRUCTION_TOOL = "rviz_plugins/ConstructionPoint";

        // Dropdown indices.
        constexpr int MODE_FULL_AUTONOMY = 0;
        constexpr int MODE_NAVIGATION_ONLY = 1;
        constexpr int TASK_CYCLE = 0;
        constexpr int TASK_EXCAVATION = 1;
        constexpr int TASK_CONSTRUCTION = 2;
        constexpr int SOURCE_MAP = 0;
        constexpr int SOURCE_ARENA = 1;

        // The same two status colours the original panel used.
        const QString GOOD = "#2e8b2e";
        const QString BAD = "#b32424";

        QString phase_text(const std::string& phase)
        {
            if (phase == "starting")
                return "Starting...";
            if (phase == "to_excavation")
                return "Driving to excavation zone";
            if (phase == "excavating")
                return "Excavating (placeholder pause)";
            if (phase == "to_construction")
                return "Driving to construction zone";
            if (phase == "depositing")
                return "Depositing (placeholder pause)";
            if (phase == "done")
                return "Done";
            return QString::fromStdString(phase);
        }

        double yaw_of(const geometry_msgs::msg::Quaternion& q)
        {
            return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }
    }  // namespace

    MissionControlPanel::MissionControlPanel(QWidget* parent)
        : rviz_common::Panel(parent)
    {
        _mode_combo = new QComboBox();
        _mode_combo->addItems({"Full Autonomy", "Navigation Only"});
        _mode_combo->setToolTip(
            "Full Autonomy: drive between the zones and run the bucket step at each one "
            "(a timed pause until the bucket action exists).\n"
            "Navigation Only: drive only.");

        _task_combo = new QComboBox();
        _task_combo->addItems({"Cycle between zones", "Go to excavation", "Go to construction"});

        _cycles_spin = new QSpinBox();
        _cycles_spin->setRange(1, 99);
        _cycles_spin->setValue(1);
        _cycles_spin->setMinimumWidth(50);
        _cycles_spin->setToolTip("One cycle: excavation -> construction -> back to excavation");

        _pause_spin = new QDoubleSpinBox();
        _pause_spin->setRange(0.5, 60.0);
        _pause_spin->setSingleStep(0.5);
        _pause_spin->setDecimals(1);
        _pause_spin->setValue(2.0);
        _pause_spin->setSuffix(" s");
        _pause_spin->setMinimumWidth(60);
        _pause_spin->setToolTip("Placeholder for the bucket action at each zone");

        _excavation_row = _build_zone_row();
        _construction_row = _build_zone_row();
        _excavation_widget = _excavation_row.source->parentWidget();
        _construction_widget = _construction_row.source->parentWidget();

        // Cycles and the bucket pause share one row.
        _run_widget = new QWidget();
        auto* run = new QHBoxLayout(_run_widget);
        run->setContentsMargins(0, 0, 0, 0);
        auto* cycles_caption = new QLabel("Cycles");
        auto* pause_caption = new QLabel("Bucket pause");
        _cycles_caption = cycles_caption;
        _pause_caption = pause_caption;
        run->addWidget(cycles_caption);
        run->addWidget(_cycles_spin, 1);
        run->addSpacing(8);
        run->addWidget(pause_caption);
        run->addWidget(_pause_spin, 1);

        auto* mission_box = new QGroupBox("Mission");
        auto* mission_form = new QFormLayout(mission_box);
        mission_form->addRow("Mode", _mode_combo);
        mission_form->addRow("Task", _task_combo);
        mission_form->addRow("Run", _run_widget);
        _task_label = mission_form->labelForField(_task_combo);
        _run_label = mission_form->labelForField(_run_widget);

        // A grid rather than a QFormLayout: Qt 5's form layout leaves a hidden row's
        // space behind (Construction on a single trip to excavation), a grid does not.
        auto* waypoint_box = new QGroupBox("Waypoints");
        auto* waypoint_grid = new QGridLayout(waypoint_box);
        auto* excavation_label = new QLabel("Excavation");
        auto* construction_label = new QLabel("Construction");
        _excavation_label = excavation_label;
        _construction_label = construction_label;
        waypoint_grid->addWidget(excavation_label, 0, 0, Qt::AlignTop);
        waypoint_grid->addWidget(_excavation_widget, 0, 1);
        waypoint_grid->addWidget(construction_label, 1, 0, Qt::AlignTop);
        waypoint_grid->addWidget(_construction_widget, 1, 1);
        waypoint_grid->setColumnStretch(1, 1);
        // The labels sit level with their dropdown, not centred on the two-line field.
        excavation_label->setMinimumHeight(_excavation_row.source->sizeHint().height());
        construction_label->setMinimumHeight(_construction_row.source->sizeHint().height());
        mission_form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        mission_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        // Never taller than their visible rows: a hidden row (e.g. Construction on a
        // single trip to excavation) would otherwise leave its space behind.
        for (auto* box : {mission_box, waypoint_box})
            box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

        _progress = new QProgressBar();
        _phase_label = new QLabel("Idle");
        _phase_label->setWordWrap(true);

        // The only styled widgets in the panel: green/red so Start and Stop can be
        // told apart at a glance, with washed-out versions when disabled.
        _start_button = new QPushButton("Start");
        _start_button->setStyleSheet(
            "QPushButton { background: #2e9e4f; color: white; border: 1px solid #237a3d;"
            " border-radius: 4px; padding: 6px; font-weight: bold; }"
            "QPushButton:hover { background: #34b058; }"
            "QPushButton:pressed { background: #257f40; }"
            "QPushButton:disabled { background: #a9d4b5; color: #eef7f1;"
            " border-color: #a9d4b5; }");
        _stop_button = new QPushButton("Stop");
        _stop_button->setStyleSheet(
            "QPushButton { background: #c93636; color: white; border: 1px solid #9e2a2a;"
            " border-radius: 4px; padding: 6px; font-weight: bold; }"
            "QPushButton:hover { background: #db4343; }"
            "QPushButton:pressed { background: #a82d2d; }"
            "QPushButton:disabled { background: #e4b1b1; color: #fbeeee;"
            " border-color: #e4b1b1; }");
        auto* buttons = new QHBoxLayout();
        buttons->addWidget(_start_button);
        buttons->addWidget(_stop_button);

        _message_label = new QLabel();
        _message_label->setWordWrap(true);
        _message_label->setStyleSheet("color: #888; font-size: 10px;");

        auto* layout = new QVBoxLayout();
        layout->addWidget(mission_box);
        layout->addWidget(waypoint_box);
        layout->addSpacing(4);
        layout->addWidget(_phase_label);
        layout->addWidget(_progress);
        layout->addLayout(buttons);
        layout->addWidget(_message_label);
        layout->addStretch(1);
        setLayout(layout);

        connect(_mode_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                &MissionControlPanel::_refresh_controls);
        connect(_task_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                &MissionControlPanel::_refresh_controls);
        connect(_cycles_spin, QOverload<int>::of(&QSpinBox::valueChanged), this,
                &MissionControlPanel::_refresh_controls);
        for (auto* row : {&_excavation_row, &_construction_row})
        {
            connect(row->source, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                    &MissionControlPanel::_on_source_changed);
        }
        connect(_excavation_row.pick_button, &QPushButton::clicked, this,
                &MissionControlPanel::_on_pick_excavation);
        connect(_construction_row.pick_button, &QPushButton::clicked, this,
                &MissionControlPanel::_on_pick_construction);
        connect(_start_button, &QPushButton::clicked, this,
                &MissionControlPanel::_on_start_clicked);
        connect(_stop_button, &QPushButton::clicked, this,
                &MissionControlPanel::_on_stop_clicked);

        _refresh_controls();
    }

    MissionControlPanel::ZoneRow MissionControlPanel::_build_zone_row()
    {
        ZoneRow row;
        auto* widget = new QWidget();
        auto* layout = new QVBoxLayout(widget);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(2);

        row.source = new QComboBox(widget);
        row.source->addItems({"Map point", "Arena auto"});
        row.source->setToolTip(
            "Map point: the point picked on the map.\n"
            "Arena auto: arena_server picks a safe point in the zone.");
        row.pick_button = new QPushButton("Pick on map", widget);
        row.pick_button->setToolTip("Click on the map, drag to set heading - like 2D Goal Pose");
        auto* top = new QHBoxLayout();
        top->addWidget(row.source, 1);
        top->addWidget(row.pick_button);
        layout->addLayout(top);

        row.point_label = new QLabel(widget);
        // Secondary detail, greyed like the other panels' small status text.
        row.point_label->setStyleSheet("color: #888; font-size: 10px;");
        layout->addWidget(row.point_label);
        return row;
    }

    // ---------------------------------------------------------------------------
    // ROS
    // ---------------------------------------------------------------------------

    void MissionControlPanel::onInitialize()
    {
        _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        _start_client = _node->create_client<StartMission>("/mission/start");
        _stop_client = _node->create_client<std_srvs::srv::Trigger>("/mission/stop");

        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        _status_sub = _node->create_subscription<MissionStatus>(
            "/mission/status", latched,
            [this](MissionStatus::ConstSharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(_mutex);
                _incoming_status = *msg;
            });
        _excavation_sub = _node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mission/excavation_point", latched,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(_mutex);
                _incoming_excavation = *msg;
            });
        _construction_sub = _node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mission/construction_point", latched,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(_mutex);
                _incoming_construction = *msg;
            });
        // Latched so the markers survive RViz reopening the display.
        _marker_pub = _node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mission/waypoint_markers", latched);

        _poll_timer = new QTimer(this);
        connect(_poll_timer, &QTimer::timeout, this, &MissionControlPanel::_poll);
        _poll_timer->start(POLL_PERIOD_MS);
        _publish_markers();
    }

    void MissionControlPanel::_poll()
    {
        std::optional<MissionStatus> status;
        std::optional<geometry_msgs::msg::PoseStamped> excavation;
        std::optional<geometry_msgs::msg::PoseStamped> construction;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            status.swap(_incoming_status);
            excavation.swap(_incoming_excavation);
            construction.swap(_incoming_construction);
        }

        bool changed = false;
        if (status)
        {
            _status = *status;
            _have_status = true;
            changed = true;
        }
        if (excavation)
        {
            _excavation_point = _from_pose(*excavation);
            _excavation_row.source->setCurrentIndex(SOURCE_MAP);
            _set_message("Excavation point set");
            _publish_markers();
            changed = true;
        }
        if (construction)
        {
            _construction_point = _from_pose(*construction);
            _construction_row.source->setCurrentIndex(SOURCE_MAP);
            _set_message("Construction point set");
            _publish_markers();
            changed = true;
        }

        const bool ready = _start_client && _start_client->service_is_ready();
        if (ready != _server_ready)
        {
            _server_ready = ready;
            changed = true;
        }

        if (_pending_start && _pending_start->wait_for(0s) == std::future_status::ready)
        {
            const auto response = _pending_start->get();
            _pending_start.reset();
            if (response->accepted)
                _set_message("Mission started", GOOD);
            else
                _set_message("Refused: " + QString::fromStdString(response->message), BAD);
            changed = true;
        }
        if (_pending_stop && _pending_stop->wait_for(0s) == std::future_status::ready)
        {
            const auto response = _pending_stop->get();
            _pending_stop.reset();
            _set_message(QString::fromStdString(response->message),
                         response->success ? QString() : BAD);
            changed = true;
        }

        if (changed)
            _refresh_controls();
    }

    // ---------------------------------------------------------------------------
    // Actions
    // ---------------------------------------------------------------------------

    void MissionControlPanel::_on_start_clicked()
    {
        if (_running() || _pending_start)
            return;
        if (!_start_client->service_is_ready())
        {
            _set_message("/mission/start is not available (navigation not up?)", BAD);
            return;
        }

        auto request = std::make_shared<StartMission::Request>();
        request->mode = _full_autonomy() ? StartMission::Request::MODE_FULL_AUTONOMY
                                         : StartMission::Request::MODE_NAVIGATION_ONLY;
        switch (_task_combo->currentIndex())
        {
        case TASK_EXCAVATION:
            request->task = StartMission::Request::TASK_GO_TO_EXCAVATION;
            break;
        case TASK_CONSTRUCTION:
            request->task = StartMission::Request::TASK_GO_TO_CONSTRUCTION;
            break;
        default:
            request->task = StartMission::Request::TASK_CYCLE;
            break;
        }
        request->cycles = static_cast<uint32_t>(_cycles_spin->value());
        request->zone_pause_s = _pause_spin->value();
        request->use_arena_excavation = _uses_arena(_excavation_row);
        request->use_arena_construction = _uses_arena(_construction_row);
        if (_excavation_point.set)
            request->excavation_point = _to_pose(_excavation_point);
        if (_construction_point.set)
            request->construction_point = _to_pose(_construction_point);

        _pending_start = _start_client->async_send_request(request);
        _set_message("Starting...");
        _refresh_controls();
    }

    void MissionControlPanel::_on_stop_clicked()
    {
        if (_pending_stop)
            return;
        if (!_stop_client->service_is_ready())
        {
            _set_message("/mission/stop is not available", BAD);
            return;
        }
        _pending_stop = _stop_client->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
        _set_message("Stopping...");
    }

    void MissionControlPanel::_on_pick_excavation()
    {
        _activate_tool(EXCAVATION_TOOL);
        _set_message("Click on the map for the excavation point, drag to set heading");
    }

    void MissionControlPanel::_on_pick_construction()
    {
        _activate_tool(CONSTRUCTION_TOOL);
        _set_message("Click on the map for the construction point, drag to set heading");
    }

    void MissionControlPanel::_on_source_changed()
    {
        _refresh_controls();
        _publish_markers();
    }

    void MissionControlPanel::_activate_tool(const QString& class_id)
    {
        auto* tools = getDisplayContext()->getToolManager();
        rviz_common::Tool* tool = nullptr;
        for (int i = 0; i < tools->numTools(); ++i)
        {
            if (tools->getTool(i)->getClassId() == class_id)
            {
                tool = tools->getTool(i);
                break;
            }
        }
        if (tool == nullptr)
            tool = tools->addTool(class_id);
        if (tool == nullptr)
        {
            _set_message("Could not load the " + class_id + " tool", BAD);
            return;
        }
        tools->setCurrentTool(tool);
    }

    // ---------------------------------------------------------------------------
    // State -> widgets
    // ---------------------------------------------------------------------------

    bool MissionControlPanel::_full_autonomy() const
    {
        return _mode_combo->currentIndex() == MODE_FULL_AUTONOMY;
    }

    bool MissionControlPanel::_uses_arena(const ZoneRow& row) const
    {
        return row.source->currentIndex() == SOURCE_ARENA;
    }

    bool MissionControlPanel::_running() const
    {
        // A latched RUNNING from a server that has since died would otherwise lock
        // the panel for good; no server means nothing is running.
        return _have_status && _server_ready &&
               _status.state == MissionStatus::STATE_RUNNING;
    }

    void MissionControlPanel::_refresh_controls()
    {
        const bool running = _running() || _pending_start.has_value();
        const bool full = _full_autonomy();
        const int task = _task_combo->currentIndex();
        const bool single_excavation = !full && task == TASK_EXCAVATION;
        const bool single_construction = !full && task == TASK_CONSTRUCTION;
        const bool cycling = !single_excavation && !single_construction;

        // Only the fields that apply to the chosen mode and task are shown.
        _task_label->setVisible(!full);
        _task_combo->setVisible(!full);
        _cycles_caption->setVisible(cycling);
        _cycles_spin->setVisible(cycling);
        _pause_caption->setVisible(full);
        _pause_spin->setVisible(full);
        _run_label->setVisible(cycling || full);
        _run_widget->setVisible(cycling || full);
        _excavation_label->setVisible(!single_construction);
        _excavation_widget->setVisible(!single_construction);
        _construction_label->setVisible(!single_excavation);
        _construction_widget->setVisible(!single_excavation);
        // Hiding rows does not shrink a group box that was laid out taller before;
        // its layout keeps the old cached size until it is invalidated.
        for (auto* box : findChildren<QGroupBox*>())
        {
            box->layout()->invalidate();
            box->updateGeometry();
        }
        if (layout() != nullptr)
            layout()->activate();

        // Locked while running, so what the panel shows is what the rover is doing.
        for (QWidget* widget :
             {static_cast<QWidget*>(_mode_combo), static_cast<QWidget*>(_task_combo),
              static_cast<QWidget*>(_cycles_spin), static_cast<QWidget*>(_pause_spin)})
        {
            widget->setEnabled(!running);
        }
        for (auto* row : {&_excavation_row, &_construction_row})
        {
            const ZonePoint& point =
                row == &_excavation_row ? _excavation_point : _construction_point;
            const bool arena = _uses_arena(*row);
            row->source->setEnabled(!running);
            row->pick_button->setEnabled(!running && !arena);
            row->point_label->setText(arena ? "arena_server picks the point" : _describe(point));
        }

        // Caught here rather than left for the server to refuse, so the panel says
        // what is missing before Start is pressed.
        QString missing;
        if (!single_construction && !_uses_arena(_excavation_row) && !_excavation_point.set)
            missing = "excavation";
        else if (!single_excavation && !_uses_arena(_construction_row) &&
                 !_construction_point.set)
            missing = "construction";

        // Status line.
        QString text;
        QString colour;
        if (!_server_ready)
        {
            text = "mission_bt_server is not running";
        }
        else if (_have_status && _status.state == MissionStatus::STATE_RUNNING)
        {
            text = "Running: " + phase_text(_status.phase);
        }
        else if (!missing.isEmpty())
        {
            text = "Pick the " + missing + " point on the map";
        }
        else if (_have_status && _status.state == MissionStatus::STATE_SUCCEEDED)
        {
            text = "Complete: " + QString::fromStdString(_status.message);
            colour = GOOD;
        }
        else if (_have_status && (_status.state == MissionStatus::STATE_FAILED ||
                                  _status.state == MissionStatus::STATE_STOPPED))
        {
            text = QString(_status.state == MissionStatus::STATE_FAILED ? "Failed: "
                                                                        : "Stopped: ") +
                   QString::fromStdString(_status.message);
            colour = _status.state == MissionStatus::STATE_FAILED ? BAD : QString();
        }
        else
        {
            text = "Idle";
        }
        _phase_label->setText(text);
        _phase_label->setStyleSheet(colour.isEmpty() ? QString()
                                                     : QString("color: %1;").arg(colour));

        // Progress.
        const bool show_cycles = _have_status && _status.cycles_target > 0 &&
                                 _status.state != MissionStatus::STATE_IDLE;
        const int target = show_cycles ? static_cast<int>(_status.cycles_target)
                                       : _cycles_spin->value();
        _progress->setVisible(cycling || show_cycles);
        _progress->setRange(0, std::max(1, target));
        _progress->setValue(show_cycles ? static_cast<int>(_status.cycles_completed) : 0);
        _progress->setFormat(QString("%1 / %2 cycles")
                                 .arg(show_cycles ? _status.cycles_completed : 0)
                                 .arg(target));

        // Start / Stop. Start is blurred as well as disabled while a mission runs, so
        // it reads as unavailable at a glance.
        _start_button->setEnabled(!running && _server_ready && missing.isEmpty());
        _stop_button->setEnabled(running);
        const bool blurred = _start_button->graphicsEffect() != nullptr;
        if (running && !blurred)
        {
            auto* blur = new QGraphicsBlurEffect(_start_button);
            blur->setBlurRadius(2.5);
            _start_button->setGraphicsEffect(blur);
        }
        else if (!running && blurred)
        {
            _start_button->setGraphicsEffect(nullptr);
        }
    }

    void MissionControlPanel::_set_message(const QString& text, const QString& colour)
    {
        _message_label->setText(text);
        _message_label->setStyleSheet(
            colour.isEmpty() ? QString("color: #888; font-size: 10px;")
                             : QString("color: %1; font-size: 10px;").arg(colour));
    }

    // ---------------------------------------------------------------------------
    // Points and markers
    // ---------------------------------------------------------------------------

    MissionControlPanel::ZonePoint
    MissionControlPanel::_from_pose(const geometry_msgs::msg::PoseStamped& pose)
    {
        ZonePoint point;
        point.set = true;
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;
        point.yaw = yaw_of(pose.pose.orientation);
        point.frame = QString::fromStdString(pose.header.frame_id);
        return point;
    }

    geometry_msgs::msg::PoseStamped MissionControlPanel::_to_pose(const ZonePoint& point)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = point.frame.toStdString();
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation.z = std::sin(point.yaw / 2.0);
        pose.pose.orientation.w = std::cos(point.yaw / 2.0);
        return pose;
    }

    QString MissionControlPanel::_describe(const ZonePoint& point)
    {
        if (!point.set)
            return "not set";
        return QString("x %1, y %2, %3%4 (%5)")
            .arg(point.x, 0, 'f', 2)
            .arg(point.y, 0, 'f', 2)
            .arg(point.yaw * 180.0 / M_PI, 0, 'f', 0)
            .arg(QString::fromUtf8("°"))
            .arg(point.frame);
    }

    void MissionControlPanel::_publish_markers()
    {
        if (!_marker_pub)
            return;

        visualization_msgs::msg::MarkerArray array;
        const auto add_zone = [&](const ZonePoint& point, bool show, int base_id,
                                  const std::string& label, float red, float green,
                                  float blue)
        {
            // disc, heading arrow, label
            for (int part = 0; part < 3; ++part)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id =
                    point.frame.isEmpty() ? "odom" : point.frame.toStdString();
                marker.ns = "mission_waypoints";
                marker.id = base_id + part;
                if (!show || !point.set)
                {
                    marker.action = visualization_msgs::msg::Marker::DELETE;
                    array.markers.push_back(marker);
                    continue;
                }
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose = _to_pose(point).pose;
                marker.color.r = red;
                marker.color.g = green;
                marker.color.b = blue;
                marker.color.a = 1.0f;
                switch (part)
                {
                case 0:
                    marker.type = visualization_msgs::msg::Marker::CYLINDER;
                    marker.scale.x = marker.scale.y = 0.7;
                    marker.scale.z = 0.03;
                    marker.color.a = 0.55f;
                    break;
                case 1:
                    marker.type = visualization_msgs::msg::Marker::ARROW;
                    marker.pose.position.z = 0.05;
                    marker.scale.x = 0.8;
                    marker.scale.y = 0.1;
                    marker.scale.z = 0.1;
                    break;
                default:
                    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    marker.pose.position.z = 0.7;
                    marker.scale.z = 0.3;
                    marker.text = label;
                    break;
                }
                array.markers.push_back(marker);
            }
        };
        // Colours match the point tools' drag arrows.
        add_zone(_excavation_point, !_uses_arena(_excavation_row), 0, "EXCAVATION", 0.96f,
                 0.65f, 0.14f);
        add_zone(_construction_point, !_uses_arena(_construction_row), 10, "CONSTRUCTION",
                 0.18f, 0.77f, 0.71f);
        _marker_pub->publish(array);
    }

    // ---------------------------------------------------------------------------
    // Config
    // ---------------------------------------------------------------------------

    void MissionControlPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("Mode", _full_autonomy() ? "full" : "navigation");
        const int task = _task_combo->currentIndex();
        config.mapSetValue("Task", task == TASK_EXCAVATION     ? "excavation"
                                   : task == TASK_CONSTRUCTION ? "construction"
                                                               : "cycle");
        config.mapSetValue("Cycles", _cycles_spin->value());
        config.mapSetValue("PauseS", _pause_spin->value());
        const auto save_zone = [&](const QString& prefix, const ZoneRow& row,
                                   const ZonePoint& point)
        {
            config.mapSetValue(prefix + "Source", _uses_arena(row) ? "arena" : "map");
            config.mapSetValue(prefix + "Set", point.set);
            config.mapSetValue(prefix + "X", point.x);
            config.mapSetValue(prefix + "Y", point.y);
            config.mapSetValue(prefix + "Yaw", point.yaw);
            config.mapSetValue(prefix + "Frame", point.frame);
        };
        save_zone("Excavation", _excavation_row, _excavation_point);
        save_zone("Construction", _construction_row, _construction_point);
    }

    void MissionControlPanel::load(const rviz_common::Config& config)
    {
        rviz_common::Panel::load(config);
        QString text;
        if (config.mapGetString("Mode", &text))
            _mode_combo->setCurrentIndex(text == "navigation" ? MODE_NAVIGATION_ONLY
                                                              : MODE_FULL_AUTONOMY);
        if (config.mapGetString("Task", &text))
        {
            _task_combo->setCurrentIndex(text == "excavation"     ? TASK_EXCAVATION
                                         : text == "construction" ? TASK_CONSTRUCTION
                                                                  : TASK_CYCLE);
        }
        int cycles = 0;
        if (config.mapGetInt("Cycles", &cycles))
            _cycles_spin->setValue(cycles);
        float pause = 0.0f;
        if (config.mapGetFloat("PauseS", &pause))
            _pause_spin->setValue(pause);

        const auto load_zone = [&](const QString& prefix, ZoneRow& row, ZonePoint& point)
        {
            QString source;
            if (config.mapGetString(prefix + "Source", &source))
                row.source->setCurrentIndex(source == "arena" ? SOURCE_ARENA : SOURCE_MAP);
            bool set = false;
            float value = 0.0f;
            if (config.mapGetBool(prefix + "Set", &set))
                point.set = set;
            if (config.mapGetFloat(prefix + "X", &value))
                point.x = value;
            if (config.mapGetFloat(prefix + "Y", &value))
                point.y = value;
            if (config.mapGetFloat(prefix + "Yaw", &value))
                point.yaw = value;
            config.mapGetString(prefix + "Frame", &point.frame);
        };
        load_zone("Excavation", _excavation_row, _excavation_point);
        load_zone("Construction", _construction_row, _construction_point);

        _refresh_controls();
        _publish_markers();
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::MissionControlPanel, rviz_common::Panel)
