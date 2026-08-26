// subscribes to a livox point cloud and removes every point whose zenith angle (up and down) 
// and azimuth angle (sideways) that falls inside a range (currently manually set range only tested and setup)

// zenith angle conventions (wrt. lidar xyz, not based on above/horizontal/below rover chassis)
// 0 = straight up 
// 90 = horizontal
// 180 = straight down

//zenith angle conventions
// 0 = directly in front of rover

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

double ZENITH_MIN_DEG;
double ZENITH_MAX_DEG;
double AZIMUTH_MIN_DEG;
double AZIMUTH_MAX_DEG;

class LidarDeadZoneFilter : public rclcpp::Node {
public:
  LidarDeadZoneFilter() : Node("lidar_dead_zone_filter") {

    ZENITH_MIN_DEG = this->declare_parameter<double>("zenith_min_deg");
    ZENITH_MAX_DEG = this->declare_parameter<double>("zenith_max_deg");
    AZIMUTH_MIN_DEG = this->declare_parameter<double>("azimuth_min_deg");
    AZIMUTH_MAX_DEG = this->declare_parameter<double>("azimuth_max_deg");

    this->declare_parameter<std::string>("input_topic");
    this->declare_parameter<std::string>("output_topic");

    std::string input_topic  = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    // subscribe to the raw livox point cloud
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10,
        std::bind(&LidarDeadZoneFilter::cloudCallback, this,
                  std::placeholders::_1));

    // publish the filtered point cloud on livox filtered
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, 10);

    RCLCPP_INFO(
        this->get_logger(),
        "dead zone: zenith [%.lf to %.lf] degrees, azimuth [%.lf to %.lf]",
        ZENITH_MIN_DEG, ZENITH_MAX_DEG, AZIMUTH_MIN_DEG, AZIMUTH_MAX_DEG);
  }

private:
  double ZENITH_MIN_DEG, ZENITH_MAX_DEG, AZIMUTH_MIN_DEG, AZIMUTH_MAX_DEG;
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // start the output message as a copy of the input, then replace its point data with only the points want to keep
    sensor_msgs::msg::PointCloud2 out = *msg;
    out.data.clear();

    // iterators that read the x, y, z fields out of the raw point data
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

      // range from the sensor origin to this point
      double range = std::sqrt(x * x + y * y + z * z);

      // Zenith angle: angle between this point and straight up (Z)
      double zenith_deg = std::acos(z / range) * 180.0 / M_PI;
      double azimuth_deg = std::atan2(y, x) * 180 / M_PI;

      // Keep the point only if its OUTSIDE the range set
      if (zenith_deg < ZENITH_MIN_DEG || zenith_deg > ZENITH_MAX_DEG) {
        const uint8_t *src = &msg->data[i * point_step];
        out.data.insert(out.data.end(), src, src + point_step);
        kept++;
      }

      // keep the point if its OUTSIDE the range set
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