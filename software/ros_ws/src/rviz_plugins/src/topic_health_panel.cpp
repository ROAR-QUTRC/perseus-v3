/// @file topic_health_panel.cpp
/// @brief Implementation of the topic health RViz panel.

#include "rviz_plugins/topic_health_panel.hpp"

#include <QHeaderView>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_plugins {
namespace {

/// @brief Column order of the table, kept in one place so the header and the
/// row filling below cannot drift apart.
enum table_columns {
  COLUMN_TOPIC = 0,
  COLUMN_RATE,
  COLUMN_AGE,
  COLUMN_COUNT,
};

} // namespace

TopicHealthPanel::TopicHealthPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  _summary_label = new QLabel("Waiting for health data...");

  _table = new QTableWidget(0, COLUMN_COUNT);
  _table->setHorizontalHeaderLabels({"Topic", "Rate (Hz)", "Age (s)"});
  _table->horizontalHeader()->setSectionResizeMode(COLUMN_TOPIC,
                                                   QHeaderView::Stretch);
  // Most Qt styles bold header sections by default. A stylesheet rule is used
  // rather than setFont(), because the style reapplies its own header font over
  // anything set that way.
  _table->horizontalHeader()->setStyleSheet(
      "QHeaderView::section { font-weight: normal; }");
  _table->verticalHeader()->setVisible(false);
  _table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  _table->setSelectionBehavior(QAbstractItemView::SelectRows);

  auto *layout = new QVBoxLayout;
  layout->addWidget(_summary_label);
  layout->addWidget(_table);
  setLayout(layout);

  _refresh_timer = new QTimer(this);
  connect(_refresh_timer, &QTimer::timeout, this, &TopicHealthPanel::_refresh);
}

void TopicHealthPanel::onInitialize() {
  _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  _subscription = _node->create_subscription<interfaces::msg::SystemHealth>(
      _topic.toStdString(), rclcpp::QoS(SUBSCRIPTION_QUEUE_DEPTH),
      [this](interfaces::msg::SystemHealth::ConstSharedPtr message) {
        _on_health(message);
      });

  _refresh_timer->start(REFRESH_PERIOD_MS);
}

void TopicHealthPanel::_on_health(
    interfaces::msg::SystemHealth::ConstSharedPtr message) {
  const std::lock_guard<std::mutex> lock(_message_mutex);
  _latest_message = std::move(message);
}

QColor TopicHealthPanel::_status_color(std::uint8_t status) {
  using interfaces::msg::TopicHealth;
  /*
    With the status column gone the row color is the only thing reporting
    status, so the amber and red are pitched to be told apart at a glance rather
    than to sit quietly behind a label. Text stays dark, so every one of these
    has to be light enough to read against.
  */
  switch (status) {
  case TopicHealth::STATUS_OK:
    return QColor(200, 240, 200);
  // Publishing, but under the expected rate by more than the tolerance.
  case TopicHealth::STATUS_SLOW:
    return QColor(255, 200, 120);
  // Nothing arriving at all, whether the publisher is silent or gone.
  case TopicHealth::STATUS_STALE:
  case TopicHealth::STATUS_NO_PUBLISHER:
    return QColor(245, 150, 150);
  default:
    return QColor(230, 230, 230);
  }
}

void TopicHealthPanel::_refresh() {
  interfaces::msg::SystemHealth::ConstSharedPtr message;
  {
    const std::lock_guard<std::mutex> lock(_message_mutex);
    message = _latest_message;
  }
  if (!message) {
    return;
  }

  using interfaces::msg::SystemHealth;
  QString overall;
  switch (message->overall_status) {
  case SystemHealth::STATUS_OK:
    overall = "OK";
    break;
  case SystemHealth::STATUS_DEGRADED:
    overall = "DEGRADED";
    break;
  default:
    overall = "ERROR";
    break;
  }
  _summary_label->setText(QString("Overall: %1    (%2 topics)")
                              .arg(overall)
                              .arg(message->topics.size()));

  _table->setRowCount(static_cast<int>(message->topics.size()));
  for (int row = 0; row < static_cast<int>(message->topics.size()); ++row) {
    const auto &topic = message->topics[static_cast<std::size_t>(row)];
    const QColor color = _status_color(topic.status);

    const QString values[COLUMN_COUNT] = {
        QString::fromStdString(topic.name),
        // Measured against expected in one cell, e.g. "197.3/200". The row
        // color is what flags a shortfall, so there is no status column to
        // read across to.
        QString("%1/%2")
            .arg(topic.measured_rate_hz, 0, 'f', 1)
            .arg(topic.expected_rate_hz, 0, 'f', 0),
        // -1 is the monitor's "never received anything" sentinel, not a real
        // age.
        topic.age_sec < 0.0 ? QString("never")
                            : QString::number(topic.age_sec, 'f', 2),
    };

    for (int column = 0; column < COLUMN_COUNT; ++column) {
      auto *item = _table->item(row, column);
      if (item == nullptr) {
        item = new QTableWidgetItem;
        _table->setItem(row, column, item);
      }
      item->setText(values[column]);
      item->setBackground(color);
      item->setForeground(QColor(20, 20, 20));
    }
  }
}

void TopicHealthPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", _topic);
}

void TopicHealthPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  config.mapGetString("Topic", &_topic);
}

} // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::TopicHealthPanel, rviz_common::Panel)
