/**
 * @file beginner_publisher.cpp
 * @brief Publishes a simple string message on the beginner_publisher topic.
 *
 * This tutorial node demonstrates how to create a timer-driven publisher in ROS 2.
 * It follows the Google C++ Style Guide.
 *
 * @author Venkata Madhav Tadavarthi
 * @date 2025-11-11
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

constexpr std::chrono::milliseconds kPublishPeriod{500};

}  // namespace

/**
 * @class MinimalPublisher
 * @brief A minimal node that periodically publishes string messages.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Initializes the publisher and timer.
   */
  MinimalPublisher()
      : rclcpp::Node("beginner_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "beginner_publisher", 10);
    timer_ = this->create_wall_timer(
        kPublishPeriod, std::bind(&MinimalPublisher::TimerCallback, this));
  }

 private:
  /**
   * @brief Publishes an incrementing greeting message.
   */
  void TimerCallback() {
    std_msgs::msg::String message;
    message.data = "This is message number: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer responsible for scheduling publishes.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  ///< String publisher.
  size_t count_;  ///< Number of messages published so far.
};

/**
 * @brief Entrypoint for the publisher node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return Exit code.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}