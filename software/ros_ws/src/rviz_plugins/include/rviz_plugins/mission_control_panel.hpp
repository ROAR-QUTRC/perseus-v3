#pragma once

/// @file mission_control_panel.hpp
/// @brief RViz panel with two buttons that send the rover to a mission zone.

#include <optional>

#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rviz_plugins {

/// @brief Dockable RViz panel: "Move to Excavation Zone" / "Move to
/// Construction Zone", each calling one of mission_bt_server's two Trigger
/// services, which runs the actual behaviour tree (request a safe point from
/// arena_server, then navigate there via nav2).
///
/// The call is asynchronous and can take a couple of minutes (an entire
/// navigation), so it is never waited on directly - that would freeze RViz's
/// GUI thread. A future is stored instead and a timer polls it for
/// completion, the same store-on-callback/read-on-timer split every other
/// panel in this plugin uses to keep Qt calls on the GUI thread, just with a
/// future instead of a subscription's latest message.
///
/// Only one mission can be in flight from this panel at a time - a second
/// click is refused while a future is pending rather than racing it. That is
/// a panel-level courtesy, not a safety property: mission_bt_server itself
/// accepts both, see its class doc for what happens if two zones are
/// requested close together from elsewhere.
class MissionControlPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  /// @brief Builds the buttons and status label. Services are created in
  /// onInitialize(), which is the first point RViz has a node to give us.
  /// @param parent Parent widget, owned by RViz.
  explicit MissionControlPanel(QWidget *parent = nullptr);

  /// @brief Creates the service clients and starts the poll timer.
  void onInitialize() override;

  /// @brief Persists the two service names into the RViz config.
  void save(rviz_common::Config config) const override;

  /// @brief Restores the two service names from the RViz config.
  void load(const rviz_common::Config &config) override;

private Q_SLOTS:
  void _on_excavation_clicked();
  void _on_construction_clicked();
  /// @brief Checks the pending future, if any, and updates the status label.
  void _poll();

private:
  /// @brief Service names unless the RViz config names others.
  static inline const QString DEFAULT_EXCAVATION_SERVICE =
      "/mission/go_to_excavation_zone";
  static inline const QString DEFAULT_CONSTRUCTION_SERVICE =
      "/mission/go_to_construction_zone";
  /// @brief Poll period. Fast enough that "arrived"/"failed" reads as
  /// immediate, cheap enough to matter not at all against a call that itself
  /// takes seconds to minutes.
  static constexpr int POLL_PERIOD_MS = 200;

  void _send_request(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
                     const QString &service_name, const QString &zone_label);

  QPushButton *_excavation_button{nullptr};
  QPushButton *_construction_button{nullptr};
  QLabel *_status_label{nullptr};
  QTimer *_poll_timer{nullptr};

  rclcpp::Node::SharedPtr _node;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _excavation_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _construction_client;

  // Only set while a request is outstanding; _poll() clears it once the
  // future is ready. std::optional rather than a default-constructed future
  // because rclcpp::Client::FutureAndRequestId has no empty state of its own.
  std::optional<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId>
      _pending_future;
  QString _pending_zone_label;

  QString _excavation_service{DEFAULT_EXCAVATION_SERVICE};
  QString _construction_service{DEFAULT_CONSTRUCTION_SERVICE};
};

} // namespace rviz_plugins
