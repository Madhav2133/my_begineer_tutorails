cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(beginner_publisher src/beginner_publisher.cpp)
add_executable(beginner_subscriber src/beginner_subscriber.cpp)

ament_target_dependencies(beginner_publisher rclcpp std_msgs example_interfaces tf2_ros geometry_msgs)
ament_target_dependencies(beginner_subscriber rclcpp std_msgs)

target_include_directories(beginner_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(beginner_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(beginner_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(beginner_subscriber PUBLIC c_std_99 cxx_std_17)

install(TARGETS beginner_publisher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS beginner_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Find Catch2 for integration tests (ROS 2 way)
  find_package(catch_ros2 REQUIRED)
  
  # ---- Build Integration Test Executable ----
  # Name it integration_test_node to match catch_ros2 discovery pattern
  add_executable(integration_test_node test/test_beginner_publisher.cpp)
  
  target_compile_features(integration_test_node PUBLIC c_std_99 cxx_std_17)

  ament_target_dependencies(integration_test_node
    rclcpp
    std_msgs
    example_interfaces
    tf2_ros
    geometry_msgs
  )

  target_link_libraries(integration_test_node
    catch_ros2::catch_ros2_with_node_main
  )

  install(TARGETS integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # Register integration test with launch file using catch_ros2
  catch_ros2_add_integration_test(PublisherIntegrationTest
    LAUNCH_FILE test_publisher.launch.py
  )

endif()

ament_package()
