#pragma once

/// @file image_stream_panel.hpp
/// @brief RViz panel rendering an image_transport stream with link statistics.

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <QComboBox>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QTimer>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rviz_plugins {

/// @brief Dockable RViz panel showing one image topic plus how well it is
/// arriving.
///
/// RViz's built-in Image display already renders any image_transport stream, so
/// this panel exists for the parts it does not show: the measured frame rate
/// and the end-to-end age of each frame, which is what tells you whether a
/// remote link is keeping up. The transport can also be switched from the
/// panel, which makes it quick to compare raw against a codec over the same
/// link.
///
/// Subscribing through image_transport rather than to Image directly means the
/// wire format is a runtime choice — "raw", "compressed", "ffmpeg", or any
/// other installed plugin — and the frame arrives already decoded either way.
///
/// The subscription callback runs on an executor thread while Qt widgets may
/// only be touched from the GUI thread, so the callback converts to a QImage
/// and stores it, and a timer on the GUI thread repaints from the latest one.
/// That also decouples repaint cost from frame rate: a stream arriving faster
/// than the panel refreshes simply has frames skipped rather than queueing work
/// onto the GUI thread.
class ImageStreamPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  /// @brief Builds the widgets. The subscription is deferred to onInitialize(),
  /// which is the first point at which RViz has a node to give us.
  /// @param parent Parent widget, owned by RViz.
  explicit ImageStreamPanel(QWidget *parent = nullptr);

  /// @brief Creates the subscription and starts the refresh timer.
  void onInitialize() override;

  /// @brief Persists the topic and transport into the RViz config.
  void save(rviz_common::Config config) const override;

  /// @brief Restores the topic and transport from the RViz config.
  void load(const rviz_common::Config &config) override;

private Q_SLOTS:
  /// @brief Repaints the image and statistics, on the GUI thread.
  void _refresh();

  /// @brief Tears down the current subscription and opens one on the topic and
  /// transport currently in the widgets.
  void _resubscribe();

private:
  /// @brief Topic the panel opens unless the RViz config names another.
  ///
  /// The base topic, without a transport suffix: image_transport appends that
  /// itself from the selected transport.
  static inline const QString DEFAULT_TOPIC = "/vision/overlay/image";
  /// @brief Transport the panel opens with.
  static inline const QString DEFAULT_TRANSPORT = "compressed";
  /// @brief Transports offered in the dropdown.
  ///
  /// The box stays editable, so a plugin that is installed but not listed here
  /// can still be typed in.
  static inline const QStringList KNOWN_TRANSPORTS = {"raw", "compressed",
                                                      "ffmpeg", "theora"};
  /// @brief Queue depth of the image subscription.
  ///
  /// Deliberately shallow: a panel that falls behind should show the newest
  /// frame, not work through a backlog of stale ones.
  static constexpr int SUBSCRIPTION_QUEUE_DEPTH = 1;
  /// @brief Repaint period, in milliseconds.
  static constexpr int REFRESH_PERIOD_MS = 50;
  /// @brief Window over which the frame rate is averaged, in seconds.
  static constexpr double RATE_WINDOW_S = 2.0;
  /// @brief Age past which the stream is called stale rather than slow, in
  /// seconds.
  static constexpr double STALE_AFTER_S = 2.0;

  /// @brief Converts an incoming frame and stores it for the next refresh.
  void _on_image(const sensor_msgs::msg::Image::ConstSharedPtr &message);

  /// @brief Converts a ROS image to a QImage that owns its pixels.
  ///
  /// The returned image is deep-copied, because the message buffer it was
  /// wrapping is released as soon as the callback returns.
  ///
  /// @param message Frame to convert.
  /// @return The converted image, or a null QImage for an encoding this panel
  /// does not handle.
  static QImage _to_qimage(const sensor_msgs::msg::Image &message);

  QLineEdit *_topic_edit{nullptr};
  QComboBox *_transport_box{nullptr};
  QLabel *_status_label{nullptr};
  QLabel *_image_label{nullptr};
  QTimer *_refresh_timer{nullptr};

  rclcpp::Node::SharedPtr _node;
  image_transport::Subscriber _subscription;

  std::mutex _frame_mutex;
  QImage _latest_frame;
  std::string _latest_encoding;
  double _latest_age_s{0.0};
  /// @brief Frames received since the rate was last recalculated.
  std::uint64_t _frames_since_tick{0};
  /// @brief Frames received in total, so a stalled stream can be told from one
  /// that never started.
  std::uint64_t _frames_total{0};

  /// @brief Start of the current rate-averaging window. GUI thread only.
  std::chrono::steady_clock::time_point _rate_window_start;
  /// @brief Frame rate over the last completed window. GUI thread only.
  double _measured_fps{0.0};

  QString _topic{DEFAULT_TOPIC};
  QString _transport{DEFAULT_TRANSPORT};
};

} // namespace rviz_plugins
