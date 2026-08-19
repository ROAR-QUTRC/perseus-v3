/// @file topic_health_panel.cpp
/// @brief Implementation of the health_check RViz panel.

#include "health_check/rviz/topic_health_panel.hpp"

#include <QHeaderView>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace health_check {
namespace {

/// @brief Column order of the table, kept in one place so the header and the
/// row filling below cannot drift apart.
enum Column {
  kTopic = 0,
  kStatus,
  kRate,
  kExpected,
  kBandwidth,
  kAge,
  kPublishers,
  kColumnCount,
};

/// @brief Formats a byte rate into the largest unit that keeps it readable.
QString formatBandwidth(double bytes_per_sec) {
  if (bytes_per_sec >= 1024.0 * 1024.0) {
    return QString::number(bytes_per_sec / (1024.0 * 1024.0), 'f', 2) +
           " MiB/s";
  }
  if (bytes_per_sec >= 1024.0) {
    return QString::number(bytes_per_sec / 1024.0, 'f', 1) + " KiB/s";
  }
  return QString::number(bytes_per_sec, 'f', 0) + " B/s";
}

} // namespace

TopicHealthPanel::TopicHealthPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  summary_label_ = new QLabel("Waiting for health data...");
  link_label_ = new QLabel("");

  table_ = new QTableWidget(0, kColumnCount);
  table_->setHorizontalHeaderLabels({"Topic", "Status", "Rate (Hz)", "Expected",
                                     "Bandwidth", "Age (s)", "Pubs"});
  table_->horizontalHeader()->setSectionResizeMode(kTopic,
                                                   QHeaderView::Stretch);
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);

  auto *layout = new QVBoxLayout;
  layout->addWidget(summary_label_);
  layout->addWidget(table_);
  layout->addWidget(link_label_);
  setLayout(layout);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &TopicHealthPanel::refresh);
}

void TopicHealthPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  subscription_ = node_->create_subscription<interfaces::msg::SystemHealth>(
      topic_.toStdString(), rclcpp::QoS(10),
      [this](interfaces::msg::SystemHealth::ConstSharedPtr message) {
        onHealth(message);
      });

  // 5 Hz is comfortably faster than the monitor's default 1 Hz publish rate
  // while staying cheap enough to leave the GUI responsive.
  refresh_timer_->start(200);
}

void TopicHealthPanel::onHealth(
    interfaces::msg::SystemHealth::ConstSharedPtr message) {
  const std::lock_guard<std::mutex> lock(message_mutex_);
  latest_message_ = std::move(message);
}

QString TopicHealthPanel::statusText(std::uint8_t status) {
  using interfaces::msg::TopicHealth;
  switch (status) {
  case TopicHealth::STATUS_OK:
    return "OK";
  case TopicHealth::STATUS_SLOW:
    return "SLOW";
  case TopicHealth::STATUS_STALE:
    return "STALE";
  case TopicHealth::STATUS_NO_PUBLISHER:
    return "NO PUBLISHER";
  default:
    return "UNKNOWN";
  }
}

QColor TopicHealthPanel::statusColour(std::uint8_t status) {
  using interfaces::msg::TopicHealth;
  switch (status) {
  case TopicHealth::STATUS_OK:
    return QColor(200, 240, 200);
  case TopicHealth::STATUS_SLOW:
    return QColor(255, 235, 180);
  case TopicHealth::STATUS_STALE:
  case TopicHealth::STATUS_NO_PUBLISHER:
    return QColor(250, 190, 190);
  default:
    return QColor(230, 230, 230);
  }
}

void TopicHealthPanel::refresh() {
  interfaces::msg::SystemHealth::ConstSharedPtr message;
  {
    const std::lock_guard<std::mutex> lock(message_mutex_);
    message = latest_message_;
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
  summary_label_->setText(QString("Overall: %1    (%2 topics)")
                              .arg(overall)
                              .arg(message->topics.size()));

  table_->setRowCount(static_cast<int>(message->topics.size()));
  for (int row = 0; row < static_cast<int>(message->topics.size()); ++row) {
    const auto &topic = message->topics[static_cast<std::size_t>(row)];
    const QColor colour = statusColour(topic.status);

    const QString values[kColumnCount] = {
        QString::fromStdString(topic.name),
        statusText(topic.status),
        QString::number(topic.measured_rate_hz, 'f', 1),
        QString::number(topic.expected_rate_hz, 'f', 1),
        formatBandwidth(topic.bandwidth_bytes_per_sec),
        // -1 is the monitor's "never received anything" sentinel, not a real
        // age.
        topic.age_sec < 0.0 ? QString("never")
                            : QString::number(topic.age_sec, 'f', 2),
        QString::number(topic.publisher_count),
    };

    for (int column = 0; column < kColumnCount; ++column) {
      auto *item = table_->item(row, column);
      if (item == nullptr) {
        item = new QTableWidgetItem;
        table_->setItem(row, column, item);
      }
      item->setText(values[column]);
      item->setBackground(colour);
      item->setForeground(QColor(20, 20, 20));
    }
  }

  const auto &link = message->link;
  QString link_text =
      QString("Link %1: ").arg(QString::fromStdString(link.interface_name));
  if (!link.interface_present) {
    link_text += "interface not found";
  } else {
    link_text += QString("rx %1  tx %2  errors %3/%4  dropped %5/%6")
                     .arg(formatBandwidth(link.rx_bytes_per_sec))
                     .arg(formatBandwidth(link.tx_bytes_per_sec))
                     .arg(link.rx_errors)
                     .arg(link.tx_errors)
                     .arg(link.rx_dropped)
                     .arg(link.tx_dropped);
  }
  if (link.ping_enabled) {
    link_text += link.reachable
                     ? QString("   ping %1: %2 ms")
                           .arg(QString::fromStdString(link.ping_host))
                           .arg(link.rtt_ms, 0, 'f', 1)
                     : QString("   ping %1: unreachable")
                           .arg(QString::fromStdString(link.ping_host));
  }
  link_label_->setText(link_text);
}

void TopicHealthPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_);
}

void TopicHealthPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  config.mapGetString("Topic", &topic_);
}

} // namespace health_check

PLUGINLIB_EXPORT_CLASS(health_check::TopicHealthPanel, rviz_common::Panel)
