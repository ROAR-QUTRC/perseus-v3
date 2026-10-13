/// @file mission_control_panel.cpp
/// @brief Implementation of MissionControlPanel.

#include "rviz_plugins/mission_control_panel.hpp"

#include <QFormLayout>
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
        constexpr int MODE_DUMP_ONLY = 2;
        constexpr int MODE_DIG_ONLY = 3;
        constexpr int MODE_FULL_AUTONOMY_TIMED = 4;
        constexpr int TASK_CYCLE = 0;
        constexpr int TASK_EXCAVATION = 1;
        constexpr int TASK_CONSTRUCTION = 2;
        constexpr int SOURCE_MAP = 0;
        constexpr int SOURCE_ARENA = 1;

        // Controller dropdown: label shown, and the controller_server plugin name the
        // behaviour tree's ControllerSelector expects (navigation.yaml's
        // controller_plugins).
        struct ControllerChoice
        {
            const char* label;
            const char* plugin;
        };
        const ControllerChoice CONTROLLERS[] = {
            {"RPP (Regulated Pure Pursuit)", "FollowPath"},
            {"DWB", "DWB"},
        };
        constexpr int CONTROLLER_RPP = 0;
        const QString CONTROLLER_TOPIC = "/controller_selector";

        // Planner dropdown, the same way: label, and the planner_server plugin name the
        // behaviour tree's PlannerSelector expects (navigation.yaml's planner_plugins).
        const ControllerChoice PLANNERS[] = {
            {"ThetaStar (any-angle)", "ThetaStar"},
            {"Lattice (heading-aware)", "Lattice"},
            {"NavFn (grid)", "GridBased"},
        };
        constexpr int PLANNER_THETA_STAR = 0;
        const QString PLANNER_TOPIC = "/planner_selector";

        // The same two status colours the original panel used.
        const QString GOOD = "#2e8b2e";
        const QString BAD = "#b32424";

        QString phase_text(const std::string& phase)
        {
            if (phase == "starting")
                return "Starting...";
            if (phase == "bucket_to_zero")
                return "Bucket: moving every joint to 0 deg";
            if (phase == "bucket_to_travel")
                return "Bucket: moving to travel pose";
            if (phase == "to_excavation")
                return "Driving to excavation zone";
            if (phase == "to_construction")
                return "Driving to construction zone";
            if (phase == "dump_lower_tip")
                return "Dumping: lowering and tipping the bucket";
            if (phase == "dump_open_jaw")
                return "Dumping: opening the jaw";
            if (phase == "dump_curl")
                return "Dumping: curling back";
            if (phase == "dump_stow")
                return "Dumping: done, stowing the bucket";
            if (phase == "dig_tilt")
                return "Digging: tipping the cutting edge down";
            if (phase == "dig_lower")
                return "Digging: lowering the arms into the regolith";
            if (phase == "dig_push")
                return "Digging: creeping forward to cut";
            if (phase == "dig_curl")
                return "Digging: curling the bucket up";
            if (phase == "dig_carry")
                return "Digging: creeping on while raising the arms";
            if (phase == "excavating")
                return "At excavation: timed stop (bucket not moved)";
            if (phase == "depositing")
                return "At construction: timed stop (bucket not moved)";
            if (phase == "done")
                return "Done";
            return QString::fromStdString(phase);
        }

        // The phases one cycle of each mission steps through, in order, as mission.xml
        // writes them. The progress display counts how many of these are done; each
        // counts the same, which is crude (a drive takes longer than a bucket move),
        // so the time estimate switches to measured cycle times once one finishes.
        const std::vector<std::string> DIG_PHASES = {"dig_tilt", "dig_lower", "dig_push",
                                                     "dig_curl", "dig_carry"};
        const std::vector<std::string> DUMP_PHASES = {"dump_lower_tip", "dump_open_jaw",
                                                      "dump_curl", "dump_stow"};
        const std::vector<std::string> FULL_CYCLE_PHASES = {
            "dig_tilt", "dig_lower", "dig_push", "dig_curl",
            "dig_carry", "to_construction", "dump_lower_tip", "dump_open_jaw",
            "dump_curl", "dump_stow", "to_excavation"};
        const std::vector<std::string> TIMED_CYCLE_PHASES = {"excavating", "to_construction",
                                                             "depositing", "to_excavation"};
        const std::vector<std::string> NAV_CYCLE_PHASES = {"to_construction",
                                                           "to_excavation"};
        const std::vector<std::string> NO_PHASES = {};

        const std::vector<std::string>& cycle_phases(const MissionStatus& status)
        {
            switch (status.mode)
            {
            case StartMission::Request::MODE_FULL_AUTONOMY:
                return FULL_CYCLE_PHASES;
            case StartMission::Request::MODE_DUMP_ONLY:
                return DUMP_PHASES;
            case StartMission::Request::MODE_DIG_ONLY:
                return DIG_PHASES;
            case StartMission::Request::MODE_FULL_AUTONOMY_TIMED:
                return TIMED_CYCLE_PHASES;
            default:
                return status.task == StartMission::Request::TASK_CYCLE ? NAV_CYCLE_PHASES
                                                                        : NO_PHASES;
            }
        }

        /// @brief "m:ss", or "h:mm:ss" past an hour.
        QString clock_text(qint64 ms)
        {
            const qint64 s = std::max<qint64>(0, (ms + 500) / 1000);
            if (s >= 3600)
                return QString("%1:%2:%3")
                    .arg(s / 3600)
                    .arg((s / 60) % 60, 2, 10, QChar('0'))
                    .arg(s % 60, 2, 10, QChar('0'));
            return QString("%1:%2").arg(s / 60).arg(s % 60, 2, 10, QChar('0'));
        }

        // Start's own look, idle. While a mission runs the button is the progress
        // bar instead: the same green filled to the fraction done over a grey track.
        const QString START_STYLE =
            "QPushButton { background: #2e9e4f; color: white; border: 1px solid #237a3d;"
            " border-radius: 4px; padding: 6px; font-weight: bold; }"
            "QPushButton:hover { background: #34b058; }"
            "QPushButton:pressed { background: #257f40; }"
            "QPushButton:disabled { background: #a9d4b5; color: #eef7f1;"
            " border-color: #a9d4b5; }";
        const QString FILL = "#2e9e4f";
        const QString TRACK = "#7d8c82";

        QString progress_style(double fraction)
        {
            QString background;
            if (fraction <= 0.0)
                background = TRACK;
            else if (fraction >= 1.0)
                background = FILL;
            else
                background = QString(
                                 "qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 %1, "
                                 "stop:%2 %1, stop:%3 %4, stop:1 %4)")
                                 .arg(FILL)
                                 .arg(fraction, 0, 'f', 4)
                                 .arg(std::min(1.0, fraction + 0.0005), 0, 'f', 4)
                                 .arg(TRACK);
            return QString(
                       "QPushButton, QPushButton:disabled { background: %1; color: white;"
                       " border: 1px solid #237a3d; border-radius: 4px; padding: 6px;"
                       " font-weight: bold; }")
                .arg(background);
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
        // Appended rather than placed beside Full Autonomy, so the indices (and the
        // MODE_* constants above) of the existing modes stay put.
        _mode_combo->addItems({"Full Autonomy", "Navigation Only", "Dump only", "Dig only",
                               "Full Autonomy (timed stops)"});
        _mode_combo->setToolTip(
            "Full Autonomy: drive between the zones, dig at excavation and dump the load "
            "at construction.\n"
            "Navigation Only: drive only.\n"
            "Dump only: no driving - run the dump sequence once, where the rover stands.\n"
            "Dig only: one dig pass from where the rover stands; it creeps ~0.6 m straight "
            "ahead while digging, so point it at the regolith first.\n"
            "Full Autonomy (timed stops): the Full Autonomy cycle, but the rover just stops\n"
            "for a set time at each zone instead of digging or dumping. The bucket is never\n"
            "moved, so this runs without bucket feedback.");

        _task_combo = new QComboBox();
        _task_combo->addItems({"Cycle between zones", "Go to excavation", "Go to construction"});

        _cycles_spin = new QSpinBox();
        _cycles_spin->setRange(1, 99);
        _cycles_spin->setValue(1);
        _cycles_spin->setMinimumWidth(50);
        _cycles_spin->setToolTip("One cycle: excavation -> construction -> back to excavation");

        _prepare_bucket_check = new QCheckBox("Move to travel pose before driving");
        _prepare_bucket_check->setChecked(true);
        _prepare_bucket_check->setToolTip(
            "Before the mission sets off: move every bucket joint to 0 deg, then to the\n"
            "travel pose (mission_bt_server's bucket_travel_*_deg, by default lift 25,\n"
            "tilt -20, jaw 0), which keeps the bucket out of the lidar's and camera's\n"
            "view of the ground ahead. Needs the bucket controller running.");

        _pause_spin = new QDoubleSpinBox();
        _pause_spin->setRange(0.5, 600.0);
        _pause_spin->setDecimals(1);
        _pause_spin->setSingleStep(1.0);
        _pause_spin->setValue(5.0);
        _pause_spin->setSuffix(" s");
        _pause_spin->setToolTip(
            "How long the rover stops at the excavation and construction zones, standing\n"
            "in for the dig and the dump. The bucket is not moved.");

        _controller_combo = new QComboBox();
        for (const auto& choice : CONTROLLERS)
            _controller_combo->addItem(choice.label, QString(choice.plugin));
        _controller_combo->setToolTip(
            "Which controller follows the planned path. Applied as soon as it is picked\n"
            "(published latched on /controller_selector, so it also survives a navigation\n"
            "restart); locked while a mission runs.\n"
            "RPP: pivots to the path heading, then pure pursuit; forward only.\n"
            "DWB: samples trajectories against the rover's full footprint; may reverse\n"
            "briefly to back out of a zone.");

        _planner_combo = new QComboBox();
        for (const auto& choice : PLANNERS)
            _planner_combo->addItem(choice.label, QString(choice.plugin));
        _planner_combo->setToolTip(
            "Which planner plans the path. Applied as soon as it is picked (published\n"
            "latched on /planner_selector, so it also survives a navigation restart);\n"
            "locked while a mission runs.\n"
            "ThetaStar: any-angle line-of-sight path; ignores heading, so the controller\n"
            "pivots onto it at the start and onto the goal heading at the end.\n"
            "Lattice: plans from the rover's heading to the goal's with 1 m arcs and\n"
            "turn-in-place, checking the full footprint; fewer pivots on the zone trips,\n"
            "weaker when it has to turn round.\n"
            "NavFn: grid Dijkstra; ignores heading, 45/90 deg steps the smoother rounds off.");

        _excavation_row = _build_zone_row();
        _construction_row = _build_zone_row();
        _excavation_widget = _excavation_row.source->parentWidget();
        _construction_widget = _construction_row.source->parentWidget();

        // Controller and the cycle count share a row: Cycles is only shown for the
        // modes that cycle, and on its own it was a whole row for one small spinbox.
        auto* controller_row = new QWidget();
        auto* controller_layout = new QHBoxLayout(controller_row);
        controller_layout->setContentsMargins(0, 0, 0, 0);
        auto* cycles_caption = new QLabel("Cycles");
        _cycles_caption = cycles_caption;
        controller_layout->addWidget(_controller_combo, 1);
        controller_layout->addSpacing(8);
        controller_layout->addWidget(cycles_caption);
        controller_layout->addWidget(_cycles_spin);

        auto* mission_box = new QGroupBox("Mission");
        auto* mission_form = new QFormLayout(mission_box);
        mission_form->addRow("Mode", _mode_combo);
        mission_form->addRow("Task", _task_combo);
        mission_form->addRow("Bucket", _prepare_bucket_check);
        mission_form->addRow("Stop for", _pause_spin);
        mission_form->addRow("Planner", _planner_combo);
        mission_form->addRow("Controller", controller_row);
        _task_label = mission_form->labelForField(_task_combo);
        _bucket_label = mission_form->labelForField(_prepare_bucket_check);
        _pause_label = mission_form->labelForField(_pause_spin);

        // A grid rather than a QFormLayout: Qt 5's form layout leaves a hidden row's
        // space behind (Construction on a single trip to excavation), a grid does not.
        auto* waypoint_box = new QGroupBox("Waypoints");
        _waypoint_box = waypoint_box;
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

        _phase_label = new QLabel("Idle");
        _phase_label->setWordWrap(true);

        // The only styled widgets in the panel: green/red so Start and Stop can be
        // told apart at a glance, with washed-out versions when disabled. While a
        // mission runs, Start doubles as its progress bar (_refresh_start_button).
        _start_button = new QPushButton("Start");
        _start_button->setStyleSheet(START_STYLE);
        _start_text = "Start";
        _start_style = START_STYLE;
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
        connect(_controller_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                &MissionControlPanel::_on_controller_changed);
        connect(_planner_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                &MissionControlPanel::_on_planner_changed);
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
        // Latched, matching nav2's ControllerSelector subscription, so bt_navigator
        // picks up the panel's choice even when navigation starts after RViz.
        _controller_pub = _node->create_publisher<std_msgs::msg::String>(
            CONTROLLER_TOPIC.toStdString(), latched);
        // Latched for the same reason, matching nav2's PlannerSelector subscription.
        _planner_pub = _node->create_publisher<std_msgs::msg::String>(
            PLANNER_TOPIC.toStdString(), latched);

        _poll_timer = new QTimer(this);
        connect(_poll_timer, &QTimer::timeout, this, &MissionControlPanel::_poll);
        _poll_timer->start(POLL_PERIOD_MS);
        _publish_markers();
        _publish_controller();
        _publish_planner();
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
            _track_progress();
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
        else if (_running())
            _refresh_start_button();  // the elapsed / remaining time keeps moving
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
        request->mode = _full_autonomy()    ? StartMission::Request::MODE_FULL_AUTONOMY
                        : _dump_only()      ? StartMission::Request::MODE_DUMP_ONLY
                        : _dig_only()       ? StartMission::Request::MODE_DIG_ONLY
                        : _timed_autonomy() ? StartMission::Request::MODE_FULL_AUTONOMY_TIMED
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
        // The timed mode never moves the bucket; the server enforces that too.
        request->prepare_bucket = !_timed_autonomy() && _prepare_bucket_check->isChecked();
        request->zone_pause_s = _timed_autonomy() ? _pause_spin->value() : 0.0;
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

    void MissionControlPanel::_on_controller_changed()
    {
        _publish_controller();
        if (_controller_pub)
            _set_message("Controller: " + _controller_combo->currentText());
    }

    void MissionControlPanel::_publish_controller()
    {
        if (!_controller_pub)
            return;
        std_msgs::msg::String msg;
        msg.data = _controller_combo->currentData().toString().toStdString();
        _controller_pub->publish(msg);
    }

    void MissionControlPanel::_on_planner_changed()
    {
        _publish_planner();
        if (_planner_pub)
            _set_message("Planner: " + _planner_combo->currentText());
    }

    void MissionControlPanel::_publish_planner()
    {
        if (!_planner_pub)
            return;
        std_msgs::msg::String msg;
        msg.data = _planner_combo->currentData().toString().toStdString();
        _planner_pub->publish(msg);
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

    bool MissionControlPanel::_dump_only() const
    {
        return _mode_combo->currentIndex() == MODE_DUMP_ONLY;
    }

    bool MissionControlPanel::_dig_only() const
    {
        return _mode_combo->currentIndex() == MODE_DIG_ONLY;
    }

    bool MissionControlPanel::_timed_autonomy() const
    {
        return _mode_combo->currentIndex() == MODE_FULL_AUTONOMY_TIMED;
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
        const bool timed = _timed_autonomy();
        // Both full autonomy modes cycle through both zones; they differ only in what
        // happens at a zone.
        const bool full = _full_autonomy() || timed;
        // Dump only and Dig only are single bucket steps where the rover stands.
        const bool bucket_step = _dump_only() || _dig_only();
        const bool navigation = !full && !bucket_step;
        const int task = _task_combo->currentIndex();
        const bool single_excavation = navigation && task == TASK_EXCAVATION;
        const bool single_construction = navigation && task == TASK_CONSTRUCTION;
        const bool cycling = !bucket_step && !single_excavation && !single_construction;

        // Only the fields that apply to the chosen mode and task are shown. Dump only
        // and Dig only drive to no zone, so they need no task, cycles or waypoints.
        _task_label->setVisible(navigation);
        _task_combo->setVisible(navigation);
        // The timed mode never moves the bucket, so it has a stop time instead of the
        // travel pose tick box.
        _bucket_label->setVisible(!timed);
        _prepare_bucket_check->setVisible(!timed);
        _pause_label->setVisible(timed);
        _pause_spin->setVisible(timed);
        _cycles_caption->setVisible(cycling);
        _cycles_spin->setVisible(cycling);
        _waypoint_box->setVisible(!bucket_step);
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
              static_cast<QWidget*>(_cycles_spin),
              static_cast<QWidget*>(_prepare_bucket_check),
              static_cast<QWidget*>(_pause_spin),
              static_cast<QWidget*>(_controller_combo),
              static_cast<QWidget*>(_planner_combo)})
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
        if (!bucket_step)
        {
            if (!single_construction && !_uses_arena(_excavation_row) &&
                !_excavation_point.set)
                missing = "excavation";
            else if (!single_excavation && !_uses_arena(_construction_row) &&
                     !_construction_point.set)
                missing = "construction";
        }

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
            if (_last_run_ms >= 0)
                text += " in " + clock_text(_last_run_ms);
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

        // Start / Stop. While a mission runs Start is disabled and shows its progress.
        _start_button->setEnabled(!running && _server_ready && missing.isEmpty());
        _stop_button->setEnabled(running);
        _refresh_start_button();
    }

    // ---------------------------------------------------------------------------
    // Progress
    // ---------------------------------------------------------------------------

    void MissionControlPanel::_track_progress()
    {
        const bool running = _status.state == MissionStatus::STATE_RUNNING;
        if (running && !_was_running)
        {
            _run_clock.start();
            _in_cycle = false;
            _work_start_ms = -1;
            _cycle_start_ms = -1;
            _cycle_ms_sum = 0;
            _cycles_timed = 0;
            _cycles_seen = 0;
            _last_run_ms = -1;
        }
        if (!running && _was_running)
            _last_run_ms = _run_clock.elapsed();
        _was_running = running;
        if (!running)
            return;

        const qint64 now = _run_clock.elapsed();
        // A cycle ends on arrival back at excavation; the next one's work starts with
        // its first phase. The phase still reads to_excavation until then.
        if (_status.cycles_completed > _cycles_seen)
        {
            // Only cycles whose start was seen are timed: a panel opened mid-mission
            // gets cycles_completed > 0 with no time to put against them.
            if (_cycle_start_ms >= 0)
            {
                _cycle_ms_sum += now - _cycle_start_ms;
                _cycles_timed += _status.cycles_completed - _cycles_seen;
            }
            _cycles_seen = _status.cycles_completed;
            _cycle_start_ms = now;
            _in_cycle = false;
        }
        const auto& phases = cycle_phases(_status);
        const auto it = std::find(phases.begin(), phases.end(), _status.phase);
        // to_excavation is both the drive out before the first cycle and the last leg
        // of every cycle; it only counts as the latter once the cycle's work began.
        const bool drive_out = _status.phase == "to_excavation" && !_in_cycle;
        if (it != phases.end() && !drive_out && !_in_cycle)
        {
            _in_cycle = true;
            if (_work_start_ms < 0)
                _work_start_ms = now;
            if (_cycle_start_ms < 0)
                _cycle_start_ms = now;
        }
    }

    std::optional<double> MissionControlPanel::_progress_fraction() const
    {
        const auto& phases = cycle_phases(_status);
        if (phases.empty())
            return std::nullopt;
        if (_status.phase == "done")
            return 1.0;
        const double target = std::max<uint32_t>(1, _status.cycles_target);
        double in_cycle = 0.0;
        const auto it = std::find(phases.begin(), phases.end(), _status.phase);
        if (_in_cycle && it != phases.end())
            in_cycle = static_cast<double>(it - phases.begin()) / phases.size();
        return std::clamp((_status.cycles_completed + in_cycle) / target, 0.0, 1.0);
    }

    std::optional<qint64> MissionControlPanel::_remaining_ms(double fraction) const
    {
        const qint64 now = _run_clock.elapsed();
        if (_cycles_timed >= 1 && _status.cycles_target > _cycles_seen)
        {
            const double mean = static_cast<double>(_cycle_ms_sum) / _cycles_timed;
            const double left = mean * (_status.cycles_target - _cycles_seen) -
                                (now - _cycle_start_ms);
            return static_cast<qint64>(std::max(0.0, left));
        }
        if (_work_start_ms >= 0 && fraction > 0.0 && fraction < 1.0)
        {
            const double worked = static_cast<double>(now - _work_start_ms);
            return static_cast<qint64>(worked / fraction * (1.0 - fraction));
        }
        return std::nullopt;
    }

    void MissionControlPanel::_refresh_start_button()
    {
        QString text = "Start";
        QString style = START_STYLE;
        if (_running())
        {
            const auto fraction = _progress_fraction();
            if (!fraction)
            {
                // A single trip: nothing to count, so how long it has been going.
                text = "Running  \u00b7  " + clock_text(_run_clock.elapsed());
                style = progress_style(0.0);
            }
            else
            {
                text = QString("%1%").arg(static_cast<int>(*fraction * 100.0));
                if (_status.cycles_target > 1)
                    text += QString("  \u00b7  cycle %1/%2")
                                .arg(std::min(_status.cycles_completed + 1,
                                              _status.cycles_target))
                                .arg(_status.cycles_target);
                const auto left = _remaining_ms(*fraction);
                text += left ? "  \u00b7  ~" + clock_text(*left) + " left"
                             : QString("  \u00b7  estimating\u2026");
                style = progress_style(*fraction);
            }
        }
        if (text != _start_text)
        {
            _start_button->setText(text);
            _start_text = text;
        }
        if (style != _start_style)
        {
            _start_button->setStyleSheet(style);
            _start_style = style;
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
        config.mapSetValue("Mode", _full_autonomy()    ? "full"
                                   : _dump_only()      ? "dump"
                                   : _dig_only()       ? "dig"
                                   : _timed_autonomy() ? "timed"
                                                       : "navigation");
        const int task = _task_combo->currentIndex();
        config.mapSetValue("Task", task == TASK_EXCAVATION     ? "excavation"
                                   : task == TASK_CONSTRUCTION ? "construction"
                                                               : "cycle");
        config.mapSetValue("Cycles", _cycles_spin->value());
        config.mapSetValue("PrepareBucket", _prepare_bucket_check->isChecked());
        config.mapSetValue("ZonePauseS", _pause_spin->value());
        config.mapSetValue("Controller", _controller_combo->currentData().toString());
        config.mapSetValue("Planner", _planner_combo->currentData().toString());
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
                                         : text == "dump"     ? MODE_DUMP_ONLY
                                         : text == "dig"      ? MODE_DIG_ONLY
                                         : text == "timed"    ? MODE_FULL_AUTONOMY_TIMED
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
        bool prepare_bucket = true;
        if (config.mapGetBool("PrepareBucket", &prepare_bucket))
            _prepare_bucket_check->setChecked(prepare_bucket);
        float pause_s = 0.0f;
        if (config.mapGetFloat("ZonePauseS", &pause_s))
            _pause_spin->setValue(pause_s);
        if (config.mapGetString("Controller", &text))
        {
            // An unknown name (e.g. a config saved with a since-removed choice) falls
            // back to RPP, the behaviour tree's own default.
            const int index = _controller_combo->findData(text);
            _controller_combo->setCurrentIndex(index >= 0 ? index : CONTROLLER_RPP);
        }
        if (config.mapGetString("Planner", &text))
        {
            // Unknown names fall back to ThetaStar, the behaviour tree's own default.
            const int index = _planner_combo->findData(text);
            _planner_combo->setCurrentIndex(index >= 0 ? index : PLANNER_THETA_STAR);
        }

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
