#include <functional>
#include <memory>
#include <fstream>
#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/dynamic_size_array.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    this->declare_parameter<int>("array_size", 1024);
    this->get_parameter("array_size", array_size_);

    subscription_ = this->create_subscription<interfaces::msg::DynamicSizeArray>(
      "mytopic", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

  ~MinimalSubscriber() {
    std::ofstream logfile;
    logfile.open("latency_log_" + std::to_string(array_size_) + ".txt", std::ios_base::app);
    for (auto latency : latency_buffer_) {
      logfile << latency << "\n";
    }
    logfile.close();
  }

private:
  void topic_callback(const interfaces::msg::DynamicSizeArray::SharedPtr msg) {
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    auto latency = timestamp - msg->timestamp;

    latency_buffer_.push_back(latency);

    RCLCPP_INFO(this->get_logger(), "I heard message latency: '%ld'", latency);
  }

  rclcpp::Subscription<interfaces::msg::DynamicSizeArray>::SharedPtr subscription_;

  int array_size_;
  std::vector<int64_t> latency_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
