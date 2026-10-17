/// @file mobility_efficiency_panel.cpp
/// @brief Implementation of the mobility efficiency RViz panel.

#include "rviz_plugins/mobility_efficiency_panel.hpp"

#include <QFrame>
#include <QPainter>
#include <QVBoxLayout>
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_plugins
{
    namespace
    {

        /// @brief Hues of the three colour-ramp anchors, in degrees.
        constexpr double RED_HUE_DEG = 0.0;
        constexpr double AMBER_HUE_DEG = 40.0;
        constexpr double GREEN_HUE_DEG = 120.0;
        /// @brief Saturation at the green end of the ramp, and at the red end.
        ///
        /// The green end is deliberately muted: a bar that sits at 95% for a whole run
        /// should not be competing for attention with the 3D view beside it. Saturation
        /// is then ramped up towards red, because hue alone does not carry the warning
        /// far enough -- at this saturation the low end comes out a mild salmon, which
        /// reads as decorative rather than as a robot failing to move.
        constexpr double GREEN_SATURATION = 0.62;
        constexpr double RED_SATURATION = 0.88;
        constexpr double RAMP_VALUE = 0.86;

        /// @brief Statuses that need a colour of their own, whatever the Qt theme.
        ///
        /// Everything else in the panel takes its colour from the widget palette, so it
        /// follows whichever theme RViz was started under -- these two do not, because
        /// "the wheels are slipping" and "the watchdog has gone quiet" have to read as
        /// warnings rather than as ordinary text. Both are mid-tone rather than bright,
        /// which is what keeps them legible against a light and a dark background
        /// alike.
        const QColor SLIPPING_COLOR(229, 57, 53);
        const QColor NO_DATA_COLOR(239, 108, 0);

        /// @brief Narrowest and widest ramp knees accepted, as a percentage.
        ///
        /// The knee divides the ramp into two segments, so it cannot sit on either end
        /// without collapsing one of them into a division by zero.
        constexpr double MIN_WARN_PERCENT = 5.0;
        constexpr double MAX_WARN_PERCENT = 95.0;

        /// @brief Linear interpolation between two hues.
        double lerp(double from, double to, double fraction)
        {
            return from + (to - from) * fraction;
        }

        /// @brief Mixes two colours, for muting a palette colour towards its
        ///        background rather than picking a grey that only suits one theme.
        /// @param from Colour at fraction 0.
        /// @param to Colour at fraction 1.
        /// @param fraction How far to move from @p from towards @p to.
        QColor blend(const QColor& from, const QColor& to, double fraction)
        {
            return QColor(static_cast<int>(lerp(from.red(), to.red(), fraction)),
                          static_cast<int>(lerp(from.green(), to.green(), fraction)),
                          static_cast<int>(lerp(from.blue(), to.blue(), fraction)));
        }

        /// @brief Accent of the velocity bars. Mid-tone blue, legible on a light and a
        /// dark theme, and nowhere on the red-amber-green ramp above, so the two
        /// readings are never mistaken for each other.
        const QColor COMMAND_COLOR(66, 133, 224);
        /// @brief A command at the end of its bar: the cap, not a fault.
        const QColor SATURATED_COLOR(239, 160, 40);

    }  // namespace

    SignedBar::SignedBar(QString caption, QString unit, double limit, QWidget* parent)
        : QWidget(parent),
          _caption(std::move(caption)),
          _unit(std::move(unit)),
          _limit(limit > 0.0 ? limit : 1.0)
    {
        setMinimumHeight(22);
    }

    void SignedBar::set_value(double value)
    {
        _value = value;
        update();
    }

    void SignedBar::set_stale(bool stale)
    {
        if (stale == _stale)
        {
            return;
        }
        _stale = stale;
        update();
    }

    void SignedBar::set_limit(double limit)
    {
        _limit = limit > 0.0 ? limit : _limit;
        update();
    }

    void SignedBar::paintEvent(QPaintEvent* /*event*/)
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        const QPalette pal = palette();
        const QColor text_color = pal.color(QPalette::WindowText);
        const QColor muted_text = blend(text_color, pal.color(QPalette::Window), 0.5);

        const QRectF track(
            CAPTION_WIDTH_PX, 4.0,
            std::max(0.0, width() - CAPTION_WIDTH_PX - VALUE_WIDTH_PX - 4.0),
            height() - 8.0);
        const double center_x = track.center().x();
        const double half_width = track.width() / 2.0;

        p.setPen(_stale ? muted_text : text_color);
        p.drawText(QRectF(0.0, 0.0, CAPTION_WIDTH_PX, height()),
                   Qt::AlignLeft | Qt::AlignVCenter, _caption);

        p.setPen(Qt::NoPen);
        p.setBrush(pal.color(QPalette::Mid));
        p.drawRoundedRect(track, CORNER_RADIUS_PX, CORNER_RADIUS_PX);

        // A stale command is shown as zero: silence from the mux means nothing is
        // commanding the rover, and an old value would say it still is.
        const double value = _stale ? 0.0 : _value;
        const double fraction = std::clamp(value / _limit, -1.0, 1.0);
        if (std::abs(fraction) > ZERO_FRACTION)
        {
            QRectF fill(fraction >= 0.0 ? center_x : center_x + fraction * half_width,
                        track.top(), std::abs(fraction) * half_width, track.height());
            const bool saturated = std::abs(value) >= _limit;
            p.setBrush(_stale ? blend(COMMAND_COLOR, pal.color(QPalette::Window), 0.6)
                              : (saturated ? SATURATED_COLOR : COMMAND_COLOR));
            // Square where it meets the center line, rounded at the free end, so the
            // fill reads as growing out of zero rather than as a floating pill.
            p.drawRoundedRect(fill, CORNER_RADIUS_PX, CORNER_RADIUS_PX);
            QRectF root = fill;
            root.setWidth(std::min(fill.width(), CORNER_RADIUS_PX * 2.0));
            if (fraction >= 0.0)
            {
                p.drawRect(root);
            }
            else
            {
                root.moveRight(center_x);
                p.drawRect(root);
            }
        }

        // The zero line, drawn over the fill so the sign is readable at a glance.
        p.setPen(QPen(muted_text, 1.0));
        p.drawLine(QPointF(center_x, track.top() - 2.0),
                   QPointF(center_x, track.bottom() + 2.0));

        p.setPen(_stale ? muted_text : text_color);
        p.drawText(QRectF(width() - VALUE_WIDTH_PX, 0.0, VALUE_WIDTH_PX, height()),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QString("%1 %2")
                       .arg(value, 0, 'f', 2)
                       .arg(_unit)
                       .replace("-", QString::fromUtf8("\u2212")));
    }

    EfficiencyBar::EfficiencyBar(QString caption, QWidget* parent)
        : QWidget(parent),
          _caption(std::move(caption))
    {
        setMinimumHeight(22);
    }

    void EfficiencyBar::set_value(double percent)
    {
        _percent = percent;
        if (percent >= 0.0)
        {
            _last_measured_percent = percent;
        }
        update();
    }

    void EfficiencyBar::set_stale(bool stale)
    {
        if (stale == _stale)
        {
            return;
        }
        _stale = stale;
        update();
    }

    void EfficiencyBar::set_warn_percent(double warn_percent)
    {
        _warn_percent = std::clamp(warn_percent, MIN_WARN_PERCENT, MAX_WARN_PERCENT);
        update();
    }

    QColor EfficiencyBar::_fill_color(double percent) const
    {
        const double clamped = std::clamp(percent, 0.0, 100.0);
        // Two segments meeting at the knee rather than one ramp across the whole
        // range, so amber lands exactly on the threshold the watchdog uses. A single
        // ramp would put amber at the midpoint, which means nothing in particular.
        const double hue_deg =
            clamped >= _warn_percent
                ? lerp(AMBER_HUE_DEG, GREEN_HUE_DEG,
                       (clamped - _warn_percent) / (100.0 - _warn_percent))
                : lerp(RED_HUE_DEG, AMBER_HUE_DEG, clamped / _warn_percent);
        // Saturation tracks the whole range rather than the two segments, so it keeps
        // rising as the value falls even once the hue has bottomed out at red.
        const double saturation =
            lerp(RED_SATURATION, GREEN_SATURATION, clamped / 100.0);
        return QColor::fromHsvF(hue_deg / 360.0, saturation, RAMP_VALUE);
    }

    void EfficiencyBar::paintEvent(QPaintEvent* /*event*/)
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        // Negative is the watchdog's not-evaluating sentinel, and also the state
        // before any message has arrived.
        const bool idle = _percent < 0.0;
        const bool inactive = idle || _stale;
        const double drawn_percent = idle ? _last_measured_percent : _percent;

        // Taken from the palette rather than hardcoded, because RViz follows the
        // desktop Qt theme: a panel built for a dark theme renders its labels almost
        // invisibly on a light one, and vice versa. Mid is the palette's recessed
        // tone, so the track reads as a groove under either theme.
        const QPalette pal = palette();
        const QColor text_color = pal.color(QPalette::WindowText);
        const QColor muted_text = blend(text_color, pal.color(QPalette::Window), 0.5);
        // Desaturated at the ramp's own brightness rather than a fixed grey, so an
        // inactive bar keeps the same weight as an active one and differs only in
        // carrying no hue -- which is the thing being signalled.
        const QColor idle_color = QColor::fromHsvF(0.0, 0.0, RAMP_VALUE);

        const QRectF track(
            CAPTION_WIDTH_PX, 4.0,
            std::max(0.0, width() - CAPTION_WIDTH_PX - VALUE_WIDTH_PX - 4.0),
            height() - 8.0);

        p.setPen(inactive ? muted_text : text_color);
        p.drawText(QRectF(0.0, 0.0, CAPTION_WIDTH_PX, height()),
                   Qt::AlignLeft | Qt::AlignVCenter, _caption);

        p.setPen(Qt::NoPen);
        p.setBrush(pal.color(QPalette::Mid));
        p.drawRoundedRect(track, CORNER_RADIUS_PX, CORNER_RADIUS_PX);

        // Greyed at its last measured width rather than emptied when inactive: "it
        // read 72% a moment ago" tells an operator something, whereas a bar that
        // silently drops to zero reads as a measurement of zero efficiency. Nothing
        // is drawn at all until a first real value has arrived.
        if (drawn_percent >= 0.0)
        {
            const double fraction = std::clamp(drawn_percent, 0.0, 100.0) / 100.0;
            QRectF fill = track;
            // Floored at a visible stub, because 0% is the worst reading this bar has
            // and drawing it as an empty track would make the most alarming state the
            // least visible one -- indistinguishable from a bar that has no data.
            fill.setWidth(std::max(MIN_FILL_WIDTH_PX, track.width() * fraction));
            p.setBrush(inactive ? idle_color : _fill_color(drawn_percent));
            p.drawRoundedRect(fill, CORNER_RADIUS_PX, CORNER_RADIUS_PX);
        }

        p.setPen(inactive ? muted_text : text_color);
        p.drawText(QRectF(width() - VALUE_WIDTH_PX, 0.0, VALUE_WIDTH_PX, height()),
                   Qt::AlignRight | Qt::AlignVCenter,
                   idle ? QString("idle") : QString("%1%").arg(_percent, 0, 'f', 0));
    }

    // ─────────────────────────────────────────────────────────────────────────────

    MobilityEfficiencyPanel::MobilityEfficiencyPanel(QWidget* parent)
        : rviz_common::Panel(parent),
          _latest_command_time(0, 0, RCL_ROS_TIME),
          _latest_message_time(0, 0, RCL_ROS_TIME)
    {
        _summary_label = new QLabel("waiting for mobility status...", this);
        // Size only, no colour: until a message arrives the label should just be
        // ordinary label text in whatever theme RViz is running under.
        _summary_label->setStyleSheet("font-size: 10px;");

        _linear_bar = new EfficiencyBar("Linear", this);
        _angular_bar = new EfficiencyBar("Angular", this);

        auto* divider = new QFrame(this);
        divider->setFrameShape(QFrame::HLine);
        divider->setFrameShadow(QFrame::Plain);
        // Faint: a separation, not a border, so it follows the theme's own line colour.
        divider->setStyleSheet("color: palette(mid);");

        _command_label = new QLabel("commanded velocity - waiting for /cmd_vel_out", this);
        _command_label->setStyleSheet("font-size: 10px;");
        _linear_command_bar =
            new SignedBar("Linear x", "m/s", DEFAULT_MAX_LINEAR_MPS, this);
        _angular_command_bar =
            new SignedBar("Yaw rate", "rad/s", DEFAULT_MAX_ANGULAR_RPS, this);

        auto* layout = new QVBoxLayout;
        layout->setContentsMargins(4, 4, 4, 4);
        layout->addWidget(_summary_label);
        layout->addWidget(_linear_bar);
        layout->addWidget(_angular_bar);
        layout->addSpacing(4);
        layout->addWidget(divider);
        layout->addWidget(_command_label);
        layout->addWidget(_linear_command_bar);
        layout->addWidget(_angular_command_bar);
        layout->addStretch(1);
        setLayout(layout);

        _refresh_timer = new QTimer(this);
        connect(_refresh_timer, &QTimer::timeout, this,
                &MobilityEfficiencyPanel::_refresh);
    }

    void MobilityEfficiencyPanel::onInitialize()
    {
        _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // Applied here rather than in load(), because RViz calls load() before the
        // panel is initialised on a saved config but not at all on a fresh one.
        _linear_bar->set_warn_percent(_warn_percent);
        _angular_bar->set_warn_percent(_warn_percent);

        _subscription = _node->create_subscription<interfaces::msg::MobilityStatus>(
            _topic.toStdString(), rclcpp::QoS(SUBSCRIPTION_QUEUE_DEPTH),
            [this](interfaces::msg::MobilityStatus::ConstSharedPtr message)
            {
                _on_status(message);
            });

        _linear_command_bar->set_limit(_max_linear);
        _angular_command_bar->set_limit(_max_angular);
        _command_subscription = _node->create_subscription<geometry_msgs::msg::TwistStamped>(
            COMMAND_TOPIC, rclcpp::QoS(SUBSCRIPTION_QUEUE_DEPTH),
            [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message)
            { _on_command(message); });

        _refresh_timer->start(REFRESH_PERIOD_MS);
    }

    void MobilityEfficiencyPanel::_on_command(
        geometry_msgs::msg::TwistStamped::ConstSharedPtr message)
    {
        // Executor thread: store only, as with the status.
        const std::lock_guard<std::mutex> lock(_command_mutex);
        _latest_command = std::move(message);
        _latest_command_time = _node->now();
    }

    void MobilityEfficiencyPanel::_on_status(
        interfaces::msg::MobilityStatus::ConstSharedPtr message)
    {
        // Executor thread: store only. Qt widgets may be touched from the GUI thread
        // alone, so the repaint happens in _refresh().
        const std::lock_guard<std::mutex> lock(_message_mutex);
        _latest_message = std::move(message);
        // Arrival time, not the message stamp: the stamp comes from the odometry the
        // verdict was computed on, so a clock offset between robot and base station
        // would show up as permanent staleness.
        _latest_message_time = _node->now();
    }

    void MobilityEfficiencyPanel::_refresh()
    {
        // The command section first: it must keep updating, and greying, when the
        // watchdog is silent and the status below returns early.
        {
            geometry_msgs::msg::TwistStamped::ConstSharedPtr command;
            rclcpp::Time command_stamp(0, 0, RCL_ROS_TIME);
            {
                const std::lock_guard<std::mutex> lock(_command_mutex);
                command = _latest_command;
                command_stamp = _latest_command_time;
            }
            const bool command_stale =
                !command || (_node->now() - command_stamp).seconds() > COMMAND_TIMEOUT_S;
            _linear_command_bar->set_stale(command_stale);
            _angular_command_bar->set_stale(command_stale);
            if (command)
            {
                _linear_command_bar->set_value(command->twist.linear.x);
                _angular_command_bar->set_value(command->twist.angular.z);
            }
            _command_label->setText(
                !command ? QString("commanded velocity - waiting for %1")
                               .arg(QString::fromStdString(COMMAND_TOPIC))
                : command_stale ? "commanded velocity - nothing commanding"
                                : "commanded velocity");
        }

        interfaces::msg::MobilityStatus::ConstSharedPtr message;
        rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
        {
            const std::lock_guard<std::mutex> lock(_message_mutex);
            message = _latest_message;
            stamp = _latest_message_time;
        }
        if (!message)
        {
            return;
        }

        const bool stale = (_node->now() - stamp).seconds() > MESSAGE_TIMEOUT_S;
        _linear_bar->set_stale(stale);
        _angular_bar->set_stale(stale);
        _linear_bar->set_value(message->linear_efficiency);
        _angular_bar->set_value(message->angular_efficiency);

        if (stale)
        {
            _set_summary("no data - watchdog silent", NO_DATA_COLOR, true);
            return;
        }
        if (message->is_slipping)
        {
            _set_summary("SLIPPING", SLIPPING_COLOR, true);
            return;
        }
        // The two ordinary states take the palette's own text colour, muted for
        // idle. Giving them colours of their own would leave the panel with four
        // competing hues and nothing to distinguish the one that matters.
        const QColor text_color = palette().color(QPalette::WindowText);
        if (!message->is_evaluating)
        {
            _set_summary("idle - nothing commanded to measure",
                         blend(text_color, palette().color(QPalette::Window), 0.4),
                         false);
            return;
        }
        _set_summary("tracking command", text_color, false);
    }

    void MobilityEfficiencyPanel::_set_summary(const QString& text,
                                               const QColor& color, bool bold)
    {
        _summary_label->setText(text);
        _summary_label->setStyleSheet(QString("color: %1; font-size: 10px; "
                                              "font-weight: %2;")
                                          .arg(color.name())
                                          .arg(bold ? "bold" : "normal"));
    }

    void MobilityEfficiencyPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("Topic", _topic);
        config.mapSetValue("WarnPercent", _warn_percent);
        config.mapSetValue("MaxLinear", _max_linear);
        config.mapSetValue("MaxAngular", _max_angular);
    }

    void MobilityEfficiencyPanel::load(const rviz_common::Config& config)
    {
        rviz_common::Panel::load(config);
        config.mapGetString("Topic", &_topic);
        config.mapGetFloat("WarnPercent", &_warn_percent);
        float max_linear = 0.0F;
        float max_angular = 0.0F;
        if (config.mapGetFloat("MaxLinear", &max_linear) && max_linear > 0.0F)
            _max_linear = max_linear;
        if (config.mapGetFloat("MaxAngular", &max_angular) && max_angular > 0.0F)
            _max_angular = max_angular;
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::MobilityEfficiencyPanel,
                       rviz_common::Panel)
