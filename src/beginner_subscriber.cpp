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
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "beginner_publisher", 10,
        std::bind(&MinimalSubscriber::MessageCallback, this, std::placeholders::_1));
  }

 private:
  /**
   * @brief Logs the content of the received message.
   *
   * @param message Incoming `std_msgs::msg::String` message pointer.
   */
  void MessageCallback(const std_msgs::msg::String::SharedPtr message) const {
    RCLCPP_INFO(this->get_logger(), "Heard: '%s'", message->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  ///< String subscription.
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
