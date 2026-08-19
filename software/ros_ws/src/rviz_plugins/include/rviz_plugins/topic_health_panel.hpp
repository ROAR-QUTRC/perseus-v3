/// @file topic_health_panel.hpp
/// @brief RViz panel showing health_check output as a table.

#ifndef HEALTH_CHECK__RVIZ__TOPIC_HEALTH_PANEL_HPP_
#define HEALTH_CHECK__RVIZ__TOPIC_HEALTH_PANEL_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <QLabel>
#include <QTableWidget>
#include <QTimer>

#include <interfaces/msg/system_health.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace health_check {

/// @brief Dockable RViz panel rendering SystemHealth as a colour-coded table.
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
  void refresh();

private:
  /// @brief Stores the incoming snapshot for the next refresh.
  void onHealth(interfaces::msg::SystemHealth::ConstSharedPtr message);

  /// @brief Background colour for a row, keyed on its TopicHealth status.
  static QColor statusColour(std::uint8_t status);

  QLabel *summary_label_{nullptr};
  QLabel *link_label_{nullptr};
  QTableWidget *table_{nullptr};
  QTimer *refresh_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<interfaces::msg::SystemHealth>::SharedPtr subscription_;

  std::mutex message_mutex_;
  interfaces::msg::SystemHealth::ConstSharedPtr latest_message_;

  QString topic_{"/health"};
};

} // namespace health_check

#endif // HEALTH_CHECK__RVIZ__TOPIC_HEALTH_PANEL_HPP_
