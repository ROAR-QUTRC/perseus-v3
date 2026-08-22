/// @file image_stream_panel.cpp
/// @brief Implementation of the image stream RViz panel.

#include "rviz_plugins/image_stream_panel.hpp"

#include <QFormLayout>
#include <QPixmap>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_plugins {
namespace {

/// @brief Smallest size the image area is allowed to collapse to, in pixels.
///
/// Without a floor the label shrinks to nothing when the panel is docked
/// narrow, and the stream looks dead rather than small.
constexpr int MINIMUM_IMAGE_HEIGHT_PX = 120;

} // namespace

ImageStreamPanel::ImageStreamPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  _topic_edit = new QLineEdit(_topic);
  // Committing on editingFinished rather than on every keystroke: resubscribing
  // per character would tear the subscription down and back up for every
  // half-typed topic name.
  connect(_topic_edit, &QLineEdit::editingFinished, this,
          &ImageStreamPanel::_resubscribe);

  _transport_box = new QComboBox;
  _transport_box->setEditable(true);
  _transport_box->addItems(KNOWN_TRANSPORTS);
  _transport_box->setCurrentText(_transport);
  // activated() rather than currentTextChanged(): the box is editable, and the
  // latter fires per keystroke, so typing a transport name would resubscribe
  // once per character and leave the failures of every prefix in the status
  // label. This pairs a dropdown pick with a committed edit instead.
  connect(_transport_box, qOverload<int>(&QComboBox::activated), this,
          &ImageStreamPanel::_resubscribe);
  connect(_transport_box->lineEdit(), &QLineEdit::editingFinished, this,
          &ImageStreamPanel::_resubscribe);

  auto *form = new QFormLayout;
  form->addRow("Topic", _topic_edit);
  form->addRow("Transport", _transport_box);

  _status_label = new QLabel("Not subscribed");

  _image_label = new QLabel;
  _image_label->setMinimumHeight(MINIMUM_IMAGE_HEIGHT_PX);
  _image_label->setAlignment(Qt::AlignCenter);
  // The label is given the panel's spare space; the pixmap itself is scaled to
  // fit in _refresh(), so that setScaledContents() cannot stretch the aspect
  // ratio.
  _image_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

  auto *layout = new QVBoxLayout;
  layout->addLayout(form);
  layout->addWidget(_status_label);
  layout->addWidget(_image_label, 1);
  setLayout(layout);

  _refresh_timer = new QTimer(this);
  connect(_refresh_timer, &QTimer::timeout, this, &ImageStreamPanel::_refresh);
}

void ImageStreamPanel::onInitialize() {
  _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  _rate_window_start = std::chrono::steady_clock::now();
  _resubscribe();
  _refresh_timer->start(REFRESH_PERIOD_MS);
}

void ImageStreamPanel::_resubscribe() {
  const QString topic = _topic_edit->text().trimmed();
  const QString transport = _transport_box->currentText().trimmed();
  if (topic.isEmpty() || transport.isEmpty()) {
    return;
  }

  // load() can run before onInitialize(), so the widgets may hold a topic well
  // before there is a node to subscribe with. Recording it is enough; the
  // subscription is opened from onInitialize().
  _topic = topic;
  _transport = transport;
  if (!_node) {
    return;
  }

  _subscription.shutdown();
  {
    const std::lock_guard<std::mutex> lock(_frame_mutex);
    _latest_frame = QImage();
    _latest_encoding.clear();
    _frames_total = 0;
    _frames_since_tick = 0;
  }
  _measured_fps = 0.0;

  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = SUBSCRIPTION_QUEUE_DEPTH;

  try {
    _subscription = image_transport::create_subscription(
        _node.get(), _topic.toStdString(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &message) {
          _on_image(message);
        },
        _transport.toStdString(), qos);
  } catch (const image_transport::TransportLoadException &e) {
    // The usual cause is the plugin for this transport not being installed in
    // RViz's environment, which is worth saying plainly rather than leaving the
    // panel silently blank.
    _status_label->setText(
        QString("Transport '%1' unavailable: %2").arg(_transport).arg(e.what()));
    _image_label->clear();
  }
}

void ImageStreamPanel::_on_image(
    const sensor_msgs::msg::Image::ConstSharedPtr &message) {
  QImage frame = _to_qimage(*message);
  const double age_s =
      (_node->now() - rclcpp::Time(message->header.stamp)).seconds();

  const std::lock_guard<std::mutex> lock(_frame_mutex);
  _latest_frame = std::move(frame);
  _latest_encoding = message->encoding;
  _latest_age_s = age_s;
  ++_frames_since_tick;
  ++_frames_total;
}

QImage ImageStreamPanel::_to_qimage(const sensor_msgs::msg::Image &message) {
  QImage::Format format = QImage::Format_Invalid;
  if (message.encoding == "bgr8") {
    format = QImage::Format_BGR888;
  } else if (message.encoding == "rgb8") {
    format = QImage::Format_RGB888;
  } else if (message.encoding == "mono8") {
    format = QImage::Format_Grayscale8;
  } else {
    return QImage();
  }

  // step is used as the stride rather than width * channels, so a publisher
  // that pads rows is handled. copy() is what makes the result own its pixels,
  // since the message buffer is gone once the callback returns.
  const QImage wrapped(message.data.data(), static_cast<int>(message.width),
                       static_cast<int>(message.height),
                       static_cast<int>(message.step), format);
  return wrapped.copy();
}

void ImageStreamPanel::_refresh() {
  QImage frame;
  std::string encoding;
  double age_s = 0.0;
  std::uint64_t frames_since_tick = 0;
  std::uint64_t frames_total = 0;
  {
    const std::lock_guard<std::mutex> lock(_frame_mutex);
    frame = _latest_frame;
    encoding = _latest_encoding;
    age_s = _latest_age_s;
    frames_total = _frames_total;
    frames_since_tick = _frames_since_tick;
    // The window is only closed below, so the counter is reset here to keep the
    // read and the reset under one lock.
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s =
        std::chrono::duration<double>(now - _rate_window_start).count();
    if (elapsed_s >= RATE_WINDOW_S) {
      _measured_fps = static_cast<double>(frames_since_tick) / elapsed_s;
      _rate_window_start = now;
      _frames_since_tick = 0;
    }
  }

  if (frames_total == 0) {
    _status_label->setText(QString("Waiting for %1 (%2)...")
                               .arg(_topic)
                               .arg(_transport));
    return;
  }

  if (frame.isNull()) {
    _status_label->setText(
        QString("Unsupported encoding '%1' — expected bgr8, rgb8 or mono8")
            .arg(QString::fromStdString(encoding)));
    return;
  }

  const QString age_text = age_s > STALE_AFTER_S
                               ? QString("STALE %1 s").arg(age_s, 0, 'f', 1)
                               : QString("%1 ms").arg(age_s * 1000.0, 0, 'f', 0);
  _status_label->setText(QString("%1x%2  %3  ·  %4 fps  ·  %5")
                             .arg(frame.width())
                             .arg(frame.height())
                             .arg(QString::fromStdString(encoding))
                             .arg(_measured_fps, 0, 'f', 1)
                             .arg(age_text));

  _image_label->setPixmap(QPixmap::fromImage(frame).scaled(
      _image_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void ImageStreamPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", _topic);
  config.mapSetValue("Transport", _transport);
}

void ImageStreamPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  if (config.mapGetString("Topic", &_topic)) {
    _topic_edit->setText(_topic);
  }
  if (config.mapGetString("Transport", &_transport)) {
    _transport_box->setCurrentText(_transport);
  }
}

} // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::ImageStreamPanel, rviz_common::Panel)
