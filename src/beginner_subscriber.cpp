/**
 * @file beginner_subscriber.cpp 
 * @brief Subscribes to beginner_publisher topic and logs incoming messages.
 *
 * This tutorial node (Optional) demonstrates how to create a subscription in ROS 2.
 * It follows the Google C++ Style Guide.
 *
 * @author
 * Venkata Madhav Tadavarthi
 * @date 2025-11-10
 */

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class MinimalSubscriber
 * @brief A minimal node that logs received string messages.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Initializes the subscriber.
   */
  MinimalSubscriber() : rclcpp::Node("beginner_subscriber") {
    this->declare_parameter<std::string>("node_tag", "default");
    node_tag_ = this->get_parameter("node_tag").as_string();

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "beginner_publisher", 10,
        std::bind(&MinimalSubscriber::MessageCallback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Subscriber ready with node tag: " << node_tag_);
  }

 private:
  /**
   * @brief Logs the content of the received message.
   *
   * @param message Incoming `std_msgs::msg::String` message pointer.
   */
  void MessageCallback(const std_msgs::msg::String::SharedPtr message) const {
    if (!message) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Received null message pointer.");
      return;
    }

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Heard (" << node_tag_ << "): " << message->data);

    if (message->data.find("Service-triggered") != std::string::npos) {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Detected service-triggered publication: " << message->data);
    } else {
      RCLCPP_DEBUG_STREAM(
          this->get_logger(),
          "Standard publication received: " << message->data);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  ///< String subscription.
  std::string node_tag_;  ///< Identifier appended to log output.
};

/**
 * @brief Entrypoint for the subscriber node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return Exit code.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
