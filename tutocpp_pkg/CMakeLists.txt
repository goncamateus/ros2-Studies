cmake_minimum_required(VERSION 3.8)
project(tutocpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(gonca_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp example_interfaces)

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp example_interfaces)

add_executable(pubsub_node src/pubsub_node.cpp)
ament_target_dependencies(pubsub_node rclcpp example_interfaces)

add_executable(server_node src/server_node.cpp)
ament_target_dependencies(server_node rclcpp example_interfaces)

add_executable(client_node src/client_node.cpp)
ament_target_dependencies(client_node rclcpp example_interfaces)

add_executable(activity_server src/activity_server.cpp)
ament_target_dependencies(activity_server rclcpp example_interfaces)

add_executable(hw_info_publisher src/hw_info_publisher.cpp)
ament_target_dependencies(hw_info_publisher rclcpp gonca_interfaces)

add_executable(hw_info_subscriber src/hw_info_subscriber.cpp)
ament_target_dependencies(hw_info_subscriber rclcpp gonca_interfaces)

add_executable(LED_server_node src/activity_04_server.cpp)
ament_target_dependencies(LED_server_node rclcpp gonca_interfaces)

add_executable(LED_client_node src/activity_04_client.cpp)
ament_target_dependencies(LED_client_node rclcpp gonca_interfaces)

add_executable(robot_publisher src/activity_05_pub.cpp)
ament_target_dependencies(robot_publisher rclcpp example_interfaces)

add_executable(turtle_spawn src/turtle_spawn.cpp)
ament_target_dependencies(turtle_spawn rclcpp example_interfaces turtlesim std_srvs)

install(TARGETS
  cpp_node
  publisher_node
  subscriber_node
  pubsub_node
  server_node
  client_node
  activity_server
  hw_info_publisher
  hw_info_subscriber
  LED_server_node
  LED_client_node
  robot_publisher
  turtle_spawn
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
