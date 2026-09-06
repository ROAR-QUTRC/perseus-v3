// subscribes to a livox point cloud and removes every point whose zenith angle
// (up and down) and azimuth angle (sideways) that falls inside a range
// (currently manually set range only tested and setup)

// zenith angle conventions (wrt. lidar xyz, not based on above/horizontal/below rover chassis) 
//0 = straight up 
//90 = horizontal 
//180 = straight down

// zenith angle conventions
// 0 = directly in front of rover

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// The zenith range to REMOVE
static const double ZENITH_MIN_DEG = 40.0;
static const double ZENITH_MAX_DEG = 70.0;
static const double AZIMUTH_MIN_DEG = 10.0;
static const double AZIMUTH_MAX_DEG = -10.0;

class LidarDeadZoneFilter : public rclcpp::Node {
public:
  LidarDeadZoneFilter() : Node("lidar_dead_zone_filter") {
    // subscribe to the raw livox point cloud
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&LidarDeadZoneFilter::cloudCallback, this,
                  std::placeholders::_1));

    // publish the filtered point cloud on livox filtered
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar/filtered", 10);

    RCLCPP_INFO(
        this->get_logger(),
        "dead zone: zenith [%.lf to %.lf] degrees, azimuth [%.lf to %.lf]",
        ZENITH_MIN_DEG, ZENITH_MAX_DEG, AZIMUTH_MIN_DEG, AZIMUTH_MAX_DEG);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Start the output message as a copy of the input, then replace its point
    // data with only the points we decide to keep
    sensor_msgs::msg::PointCloud2 out = *msg;
    out.data.clear();

    // Iterators that read the x, y, z fields out of the raw point data
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    size_t num_points = msg->width * msg->height;
    size_t point_step = msg->point_step; // bytes per point
    size_t kept = 0;

    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;

      // Range from the sensor origin to this point
      double range = std::sqrt(x * x + y * y + z * z);

      // Zenith angle: angle between this point and straight up (+Z).
      double zenith_deg = std::acos(z / range) * 180.0 / M_PI;
      double azimuth_deg = std::atan2(y, x) * 180 / M_PI;

      // Keep the point only if it's OUTSIDE the cutout range
      if (zenith_deg < ZENITH_MIN_DEG || zenith_deg > ZENITH_MAX_DEG) {
        const uint8_t *src = &msg->data[i * point_step];
        out.data.insert(out.data.end(), src, src + point_step);
        kept++;
      }

      if (!(azimuth_deg > AZIMUTH_MAX_DEG && azimuth_deg < AZIMUTH_MIN_DEG)) {
        const uint8_t *src = &msg->data[i * point_step];
        out.data.insert(out.data.end(), src, src + point_step);
        kept++;
      }
    }

    // Update the size fields to match how many points actually kept
    out.width = kept;
    out.row_step = point_step * kept;
    out.height = 1;

    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDeadZoneFilter>());
  rclcpp::shutdown();
  return 0;
}