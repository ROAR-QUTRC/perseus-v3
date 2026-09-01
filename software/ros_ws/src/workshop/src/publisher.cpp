#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node {

public:
  PublisherNode() : Node("publisher_node"), count_(0) {
    RCLCPP_INFO(this->get_logger(), "CREATED PUBLISHER!!!");

    declare_parameter<std::string>("publish_message", "Hello, world!");
    declare_parameter<int>("publish_period_ms", 500);

    publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);

    create_timer_from_param();

    parameter_change = add_on_set_parameters_callback(
        std::bind(&MyNode::on_param_change, this, std::placeholders::_1));
  }

private:
  void create_timer_from_param() {
    int period_ms = get_parameter("publish_period_ms").as_int();
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&PublisherNode::timer_callback, this));
  }

  // Callback
  void timer_callback() {
    auto msg = std_msgs::msg::String();
    auto text = get_parameter("publish_message").as_string();
    msg.data = text + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  // Variables
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}