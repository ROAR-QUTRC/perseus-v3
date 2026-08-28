#pragma once

/// @file topic_health_panel.hpp
/// @brief RViz panel showing health_check output as a table.

#include <memory>
#include <mutex>
#include <string>

#include <QLabel>
#include <QTableWidget>
#include <QTimer>

#include <interfaces/msg/system_health.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace rviz_plugins {

/// @brief Dockable RViz panel rendering SystemHealth as a color-coded table.
///
/// RViz has no generic table display for custom messages, so the table is built
/// here as a Qt widget rather than assembled from existing displays.
///
/// The subscription callback arrives on an executor thread while Qt widgets may
/// only be touched from the GUI thread, so the callback only stores the latest
/// message and a timer on the GUI thread repaints from it. That also decouples
/// repaint cost from publish rate: a monitor publishing faster than the panel
/// refreshes simply has some snapshots skipped rather than queueing work onto
/// the GUI thread.
class TopicHealthPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  /// @brief Builds the widgets. The subscription is deferred to onInitialize(),
  /// which is the first point at which RViz has a node to give us.
  /// @param parent Parent widget, owned by RViz.
  explicit TopicHealthPanel(QWidget *parent = nullptr);

  /// @brief Creates the subscription and starts the refresh timer.
  void onInitialize() override;

  /// @brief Persists the watched topic into the RViz config.
  void save(rviz_common::Config config) const override;

  /// @brief Restores the watched topic from the RViz config.
  void load(const rviz_common::Config &config) override;

private Q_SLOTS:
  /// @brief Repaints the table from the most recent message, on the GUI thread.
  void _refresh();

private:
  /// @brief Topic the panel subscribes to unless the RViz config names another.
  static inline const QString DEFAULT_TOPIC = "/health_check/health";
  /// @brief Queue depth of the health subscription.
  static constexpr int SUBSCRIPTION_QUEUE_DEPTH = 10;
  /// @brief Repaint period, in milliseconds.
  ///
  /// 5 Hz is comfortably faster than the monitor's default 1 Hz publish rate
  /// while staying cheap enough to leave the GUI responsive.
  static constexpr int REFRESH_PERIOD_MS = 200;

  /// @brief Stores the incoming snapshot for the next refresh.
  void _on_health(interfaces::msg::SystemHealth::ConstSharedPtr message);

  /// @brief Background color for a row, keyed on its TopicHealth status.
  static QColor _status_color(std::uint8_t status);

  QLabel *_summary_label{nullptr};
  QTableWidget *_table{nullptr};
  QTimer *_refresh_timer{nullptr};

  rclcpp::Node::SharedPtr _node;
  rclcpp::Subscription<interfaces::msg::SystemHealth>::SharedPtr _subscription;

  std::mutex _message_mutex;
  interfaces::msg::SystemHealth::ConstSharedPtr _latest_message;

  QString _topic{DEFAULT_TOPIC};
};

} // namespace rviz_plugins
