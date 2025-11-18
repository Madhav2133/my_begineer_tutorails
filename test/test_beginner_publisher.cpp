#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief Level-2 integration test for beginner_publisher topic output.
 *
 * This test checks whether the MinimalPublisher node publishes at least one
 * message on the "beginner_publisher" topic within a timeout.
 */

TEST_CASE("beginner_publisher topic test", "beginner_publisher") {

  // Create a test node
  auto test_node = rclcpp::Node::make_shared("beginner_publisher_test_node");

  bool received_message = false;
  std::string received_data;

  // Create subscription to the topic your node publishes
  auto sub = test_node->create_subscription<std_msgs::msg::String>(
      "beginner_publisher",
      10,
      [&](const std_msgs::msg::String::SharedPtr msg) {
        received_message = true;
        received_data = msg->data;
        RCLCPP_INFO(test_node->get_logger(),
                    "Test received: \"%s\"", msg->data.c_str());
      });

  // Spin + timeout loop
  rclcpp::Rate rate(20.0);  // 20 Hz checking
  auto start = test_node->get_clock()->now();
  auto timeout = rclcpp::Duration::from_seconds(3.0);  // 3-second window

  while (!received_message &&
         (test_node->get_clock()->now() - start) < timeout) {
    rclcpp::spin_some(test_node);
    rate.sleep();
  }

  // REQUIRE ensures test stops immediately on failure
  REQUIRE(received_message);

  // CHECK ensures additional non-fatal validation
  CHECK(received_data.find("Hey") != std::string::npos);
}
