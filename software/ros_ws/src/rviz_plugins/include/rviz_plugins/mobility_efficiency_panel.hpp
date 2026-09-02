#pragma once

/// @file mobility_efficiency_panel.hpp
/// @brief RViz panel showing mobility efficiency as two colour-coded bars.

#include <mutex>

#include <QColor>
#include <QLabel>
#include <QPalette>
#include <QString>
#include <QTimer>
#include <QWidget>

#include <interfaces/msg/mobility_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace rviz_plugins {

/// @brief One labelled horizontal bar reading 0-100%, coloured by its value.
///
/// A bar rather than a number because the value being watched is a margin, not
/// a reading: an operator needs to see it dropping towards the slip threshold
/// while driving, which is a shape rather than a figure to be read.
///
/// Instantiated once per channel by MobilityEfficiencyPanel, so the two bars
/// cannot disagree about how a value maps to a colour.
class EfficiencyBar : public QWidget {
  Q_OBJECT

public:
  /// @brief Builds an empty bar in its idle state.
  /// @param caption Channel name drawn at the left, e.g. "Linear".
  /// @param parent Parent widget.
  explicit EfficiencyBar(QString caption, QWidget *parent = nullptr);

  /// @brief Sets the value to draw and repaints.
  /// @param percent Efficiency in 0..100, or negative for a channel that is
  ///        not being evaluated (see MobilityStatus.msg).
  void set_value(double percent);

  /// @brief Greys the bar without discarding its last value, for when the
  ///        status topic has gone quiet.
  /// @param stale Whether the panel has stopped hearing from the watchdog.
  void set_stale(bool stale);

  /// @brief Sets the efficiency at which the colour ramp reaches amber.
  /// @param warn_percent Knee of the ramp, clamped into a usable range.
  void set_warn_percent(double warn_percent);

private:
  /// @brief Width reserved for the caption column, in pixels.
  static constexpr int CAPTION_WIDTH_PX = 62;
  /// @brief Width reserved for the value readout, in pixels.
  static constexpr int VALUE_WIDTH_PX = 52;
  /// @brief Corner radius of the track and fill, in pixels.
  static constexpr double CORNER_RADIUS_PX = 3.0;
  /// @brief Narrowest fill drawn for a channel that has a value, in pixels.
  static constexpr double MIN_FILL_WIDTH_PX = 7.0;

  /// @brief Colour for an efficiency, ramping red - amber - green.
  /// @param percent Efficiency in 0..100.
  QColor _fill_color(double percent) const;

  void paintEvent(QPaintEvent *event) override;

  QString _caption;
  /// @brief Value most recently handed to set_value(). Negative until a value
  /// has ever arrived, which is also how a not-evaluating channel is
  /// represented, so the two render the same way -- in both cases there is
  /// nothing currently being measured to report.
  double _percent{-1.0};
  /// @brief Last value that was an actual measurement, kept so an idle bar can
  /// grey at the width it last read rather than collapsing to zero, which
  /// would look like a measurement of no efficiency at all.
  double _last_measured_percent{-1.0};
  bool _stale{false};
  double _warn_percent{60.0};
};

/// @brief Dockable RViz panel drawing the watchdog's two efficiency channels.
///
/// The subscription callback arrives on an executor thread while Qt widgets may
/// only be touched from the GUI thread, so the callback only stores the latest
/// message and a timer on the GUI thread repaints from it.
///
/// The timer is also what makes staleness visible. The watchdog publishes at
/// half-second intervals, so a repaint driven by the callback could not tell a
/// slow topic from a dead one: when the node exits or the link drops the
/// callback simply stops firing, leaving the bars asserting a value that
/// stopped updating minutes ago.
class MobilityEfficiencyPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  /// @brief Builds the widgets. The subscription is deferred to onInitialize(),
  /// which is the first point at which RViz has a node to give us.
  /// @param parent Parent widget, owned by RViz.
  explicit MobilityEfficiencyPanel(QWidget *parent = nullptr);

  /// @brief Creates the subscription and starts the refresh timer.
  void onInitialize() override;

  /// @brief Persists the watched topic and ramp knee into the RViz config.
  void save(rviz_common::Config config) const override;

  /// @brief Restores the watched topic and ramp knee from the RViz config.
  void load(const rviz_common::Config &config) override;

private Q_SLOTS:
  /// @brief Repaints the bars from the most recent message, on the GUI thread.
  void _refresh();

private:
  /// @brief Topic the panel subscribes to unless the RViz config names another.
  static inline const QString DEFAULT_TOPIC = "/watchdog/mobility_status";
  /// @brief Queue depth of the status subscription.
  static constexpr int SUBSCRIPTION_QUEUE_DEPTH = 10;
  /// @brief Repaint period, in milliseconds.
  static constexpr int REFRESH_PERIOD_MS = 200;
  /// @brief Age at which the last message is treated as no longer current.
  ///
  /// Four times the watchdog's default publish_period_s, so an ordinary late
  /// message does not flicker the panel into "no data" but a stopped publisher
  /// is called out within a couple of seconds.
  static constexpr double MESSAGE_TIMEOUT_S = 2.0;
  /// @brief Default efficiency at which the colour ramp reaches amber.
  ///
  /// Matches the watchdog's default slip_ratio_threshold of 0.4, so the bars
  /// turn amber at the same point the watchdog starts calling it slip. Keep the
  /// two in step if either is retuned.
  static constexpr double DEFAULT_WARN_PERCENT = 60.0;

  /// @brief Stores the incoming status for the next refresh.
  void _on_status(interfaces::msg::MobilityStatus::ConstSharedPtr message);

  /// @brief Sets the summary line's text and appearance.
  /// @param text Status wording to show.
  /// @param color Text colour.
  /// @param bold Whether to weight the text, reserved for the warning states.
  void _set_summary(const QString &text, const QColor &color, bool bold);

  QLabel *_summary_label{nullptr};
  EfficiencyBar *_linear_bar{nullptr};
  EfficiencyBar *_angular_bar{nullptr};
  QTimer *_refresh_timer{nullptr};

  rclcpp::Node::SharedPtr _node;
  rclcpp::Subscription<interfaces::msg::MobilityStatus>::SharedPtr
      _subscription;

  std::mutex _message_mutex;
  interfaces::msg::MobilityStatus::ConstSharedPtr _latest_message;
  rclcpp::Time _latest_message_time;

  QString _topic{DEFAULT_TOPIC};
  float _warn_percent{static_cast<float>(DEFAULT_WARN_PERCENT)};
};

} // namespace rviz_plugins
