/**
 * @file beginner_publisher.cpp
 * @brief Publishes a simple string message on the beginner_publisher topic.
 *
 * This tutorial node demonstrates how to create a timer-driven publisher in ROS 2.
 * It follows the Google C++ Style Guide.
 *
 * @author Venkata Madhav Tadavarthi
 * @date 2025-11-10
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "example_interfaces/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

constexpr std::chrono::milliseconds kDefaultPublishPeriod{500};

}  // namespace

/**
 * @class MinimalPublisher
 * @brief A minimal node that periodically publishes string messages.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Initializes the publisher, timer, and service.
   */
  MinimalPublisher()
      : rclcpp::Node("beginner_publisher"), base_message_("Hey! How are you?") {
    this->declare_parameter<int64_t>(
        "publish_period_ms", kDefaultPublishPeriod.count());
    const auto period_ms = this->get_parameter("publish_period_ms").as_int();
    if (period_ms <= 0) {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "Invalid publish_period_ms parameter: " << period_ms
                                                 << ". Using default.");
      publish_period_ = kDefaultPublishPeriod;
    } else {
      publish_period_ = std::chrono::milliseconds(period_ms);
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "beginner_publisher", 10);
    timer_ = this->create_wall_timer(
        publish_period_, std::bind(&MinimalPublisher::TimerCallback, this));
    service_ = this->create_service<example_interfaces::srv::SetBool>(
        "toggle_message",
        std::bind(
            &MinimalPublisher::HandleToggleMessage, this,
            std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Publisher initialized with period (ms): " << publish_period_.count());
  }

 private:
  /**
   * @brief Publishes an incrementing greeting message.
   */
  void TimerCallback() {
    std_msgs::msg::String message;
    message.data = base_message_ + " #" + std::to_string(count_++);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing message: " << message.data);
    publisher_->publish(message);
  }

  /**
   * @brief Handles the toggle_message service requests.
   *
   * @param request Service request with boolean flag.
   * @param response Service response indicating new state.
   */
  void HandleToggleMessage(
      const example_interfaces::srv::SetBool::Request::SharedPtr request,
      const example_interfaces::srv::SetBool::Response::SharedPtr response) {
    if (request->data) {
      base_message_ = "This is a service-triggered message ";
      response->message = "Base message set to service-triggered message.";
    } else {
      base_message_ = "Hey! How are you?";
      response->message = "Base message reset to default.";
    }
    response->success = true;
    RCLCPP_WARN_STREAM(this->get_logger(), "Service updated base message to: "
                                               << base_message_);
  }

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer responsible for scheduling publishes.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  ///< String publisher.
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;  ///< Toggle service.
  std::chrono::milliseconds publish_period_;  ///< Period between publishes.
  std::string base_message_;  ///< Current base message string.
  size_t count_ = 0;  ///< Number of messages published so far.
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