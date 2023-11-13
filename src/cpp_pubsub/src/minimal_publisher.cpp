#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/dynamic_size_array.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<interfaces::msg::DynamicSizeArray>("mytopic", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::timer_callback, this));

    this->declare_parameter<int>("array_size", 1024);
    this->get_parameter("array_size", array_size_);
  }

private:
  void timer_callback() {
    auto message = interfaces::msg::DynamicSizeArray();
    message.data.resize(array_size_);
    RCLCPP_INFO(this->get_logger(), "Publishing Message %d", count_++);

    auto now = std::chrono::system_clock::now();
    message.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    publisher_->publish(std::move(message));
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::DynamicSizeArray>::SharedPtr publisher_;
  int count_;
  int array_size_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
