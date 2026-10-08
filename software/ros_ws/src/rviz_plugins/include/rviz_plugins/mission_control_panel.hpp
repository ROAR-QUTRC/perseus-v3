#pragma once

/// @file mission_control_panel.hpp
/// @brief RViz panel that configures, starts and stops a mission on mission_bt_server.

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QTimer>
#include <QWidget>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/mission_status.hpp>
#include <interfaces/srv/start_mission.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rviz_plugins
{

    /// @brief Mission Control: pick an operation mode and the two zone waypoints,
    /// set the number of cycles, then Start / Stop.
    ///
    ///   Operation mode   Full Autonomy (drive + the bucket step at each zone: dig
    ///                    at excavation, dump at construction), Navigation Only (drive only: cycle, or a
    ///                    single trip to one zone), Dump only (no driving: run the
    ///                    dump sequence once, where the rover stands) or Dig only (one
    ///                    dig pass from where the rover stands, creeping straight ahead)
    ///                    or Full Autonomy (timed stops): Full Autonomy's drive cycle,
    ///                    stopping for a set time at each zone instead of moving the
    ///                    bucket, for a rover with no bucket feedback.
    ///   Zone waypoints   per zone, a point picked on the map with the Excavation /
    ///                    Construction Point tools (click-drag like 2D Goal Pose; a
    ///                    labelled marker stays on the map), or arena_server's own
    ///                    safe point for that zone.
    ///   Cycles           excavation -> construction -> back to excavation, counted
    ///                    on the way back; progress comes from /mission/status.
    ///   Bucket           optionally move the bucket to every joint 0 deg and then
    ///                    to its travel pose before the rover sets off
    ///                    (StartMission.prepare_bucket).
    ///   Planner          which planner_server plugin plans the path (ThetaStar,
    ///                    Lattice or NavFn), published latched on /planner_selector,
    ///                    which the behaviour tree's PlannerSelector reads every tick.
    ///   Controller       which controller_server plugin follows the path (RPP or
    ///                    DWB), published latched on /controller_selector, which the
    ///                    behaviour tree's ControllerSelector reads every tick.
    ///
    /// Start calls /mission/start (interfaces/StartMission), which returns as soon as
    /// the mission is accepted; everything after that -- phase, cycle count, outcome
    /// -- is read off the latched /mission/status, so a panel opened mid-mission (or a
    /// second base station) shows the same thing. Stop calls /mission/stop.
    ///
    /// While a mission runs, Start is disabled and blurred and the configuration is
    /// locked, so what the panel shows is what the rover is doing.
    ///
    /// Threading: ROS callbacks only store under _mutex; the Qt timer applies them to
    /// widgets on the GUI thread, the same split the other panels here use.
    class MissionControlPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit MissionControlPanel(QWidget* parent = nullptr);

        /// @brief Creates the ROS clients, subscriptions and marker publisher.
        void onInitialize() override;

        /// @brief Persists mode, cycles, the bucket tick box, planner, controller, sources
        /// and picked points.
        void save(rviz_common::Config config) const override;

        /// @brief Restores the above and redraws the waypoint markers.
        void load(const rviz_common::Config& config) override;

    private Q_SLOTS:
        void _on_start_clicked();
        void _on_stop_clicked();
        void _on_pick_excavation();
        void _on_pick_construction();
        /// @brief Redraws the markers when a zone's source changes.
        void _on_source_changed();
        /// @brief Publishes the newly chosen controller on /controller_selector.
        void _on_controller_changed();
        /// @brief Publishes the newly chosen planner on /planner_selector.
        void _on_planner_changed();
        /// @brief Re-derives every widget's visibility/enabled state from the
        /// current selections and mission state.
        void _refresh_controls();
        /// @brief Applies stored ROS data and completes pending service calls.
        void _poll();

    private:
        /// @brief One zone's waypoint as the panel keeps it.
        struct ZonePoint
        {
            bool set{false};
            double x{0.0};
            double y{0.0};
            double yaw{0.0};
            QString frame;
        };

        /// @brief Widgets of one zone's row: where its point comes from, the Pick
        /// button, and the picked point.
        struct ZoneRow
        {
            QComboBox* source{nullptr};
            QPushButton* pick_button{nullptr};
            QLabel* point_label{nullptr};
        };

        static constexpr int POLL_PERIOD_MS = 100;

        /// @brief Builds one zone's source dropdown, Pick button and point label.
        ZoneRow _build_zone_row();

        /// @brief Activates a zone's point tool in RViz's tool manager, adding it to
        /// the toolbar first if the config does not already have it.
        void _activate_tool(const QString& class_id);

        static ZonePoint _from_pose(const geometry_msgs::msg::PoseStamped& pose);
        static geometry_msgs::msg::PoseStamped _to_pose(const ZonePoint& point);
        static QString _describe(const ZonePoint& point);

        /// @brief Republishes the waypoint markers for whichever zones use a picked
        /// point, and deletes the rest.
        void _publish_markers();

        /// @brief Publishes the selected controller's plugin name, latched.
        void _publish_controller();

        /// @brief Publishes the selected planner's plugin name, latched.
        void _publish_planner();

        bool _full_autonomy() const;
        bool _dump_only() const;
        bool _dig_only() const;
        bool _timed_autonomy() const;
        bool _uses_arena(const ZoneRow& row) const;
        bool _running() const;
        void _set_message(const QString& text, const QString& colour = QString());

        /// @brief Updates the run's timing (start, cycle boundaries) from a new status.
        void _track_progress();
        /// @brief How far through the running mission is, 0..1, from the cycles done
        /// and the phases done in the current cycle; nullopt for a single trip, which
        /// has no steps to count.
        std::optional<double> _progress_fraction() const;
        /// @brief Estimated milliseconds left: the mean measured cycle time once a
        /// cycle has finished, extrapolated from the phases done before that; nullopt
        /// while there is nothing to go on yet.
        std::optional<qint64> _remaining_ms(double fraction) const;
        /// @brief Start as a button when idle, as the progress bar while running.
        void _refresh_start_button();

        // ---- widgets ----
        QComboBox* _mode_combo{nullptr};
        QComboBox* _task_combo{nullptr};
        QSpinBox* _cycles_spin{nullptr};
        QCheckBox* _prepare_bucket_check{nullptr};
        // Full Autonomy (timed stops) only: seconds stopped at each zone.
        QDoubleSpinBox* _pause_spin{nullptr};
        QComboBox* _controller_combo{nullptr};
        QComboBox* _planner_combo{nullptr};
        ZoneRow _excavation_row;
        ZoneRow _construction_row;
        // Form rows hidden or shown with the mode: each field's label and its widget.
        QWidget* _task_label{nullptr};
        QWidget* _bucket_label{nullptr};
        QWidget* _pause_label{nullptr};
        // Cycles caption, beside its spinbox on the Controller row; both hidden when
        // the mode does not cycle.
        QWidget* _cycles_caption{nullptr};
        QWidget* _excavation_label{nullptr};
        QWidget* _excavation_widget{nullptr};
        QWidget* _construction_label{nullptr};
        QWidget* _construction_widget{nullptr};
        QWidget* _waypoint_box{nullptr};
        QLabel* _phase_label{nullptr};
        QPushButton* _start_button{nullptr};
        QPushButton* _stop_button{nullptr};
        QLabel* _message_label{nullptr};
        QTimer* _poll_timer{nullptr};

        // ---- state ----
        ZonePoint _excavation_point;
        ZonePoint _construction_point;
        interfaces::msg::MissionStatus _status;
        bool _have_status{false};
        bool _server_ready{false};

        // ---- run timing, for the progress display; wall clock ----
        QElapsedTimer _run_clock;  // started when a mission is seen running
        bool _was_running{false};
        bool _in_cycle{false};       // a phase of the current cycle's work has begun
        qint64 _work_start_ms{-1};   // first in-cycle phase (after the drive out)
        qint64 _cycle_start_ms{-1};  // start of the current cycle's work
        qint64 _cycle_ms_sum{0};     // total time of the cycles timed so far
        uint32_t _cycles_timed{0};   // cycles whose start and end were both seen
        uint32_t _cycles_seen{0};    // cycles_completed at the last update
        qint64 _last_run_ms{-1};     // duration of the last mission, once it ends
        QString _start_text;         // what the Start button last showed, to skip
        QString _start_style;        // restyling it on every poll

        // ---- ROS ----
        rclcpp::Node::SharedPtr _node;
        rclcpp::Client<interfaces::srv::StartMission>::SharedPtr _start_client;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _stop_client;
        rclcpp::Subscription<interfaces::msg::MissionStatus>::SharedPtr _status_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _excavation_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _construction_sub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _controller_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _planner_pub;

        std::optional<rclcpp::Client<interfaces::srv::StartMission>::FutureAndRequestId>
            _pending_start;
        std::optional<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId>
            _pending_stop;

        // Written by ROS callbacks, taken by _poll().
        std::mutex _mutex;
        std::optional<interfaces::msg::MissionStatus> _incoming_status;
        std::optional<geometry_msgs::msg::PoseStamped> _incoming_excavation;
        std::optional<geometry_msgs::msg::PoseStamped> _incoming_construction;
    };

}  // namespace rviz_plugins
