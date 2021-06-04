cmake_minimum_required(VERSION 3.5)
project(cpp_turtlesim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
### things Luqman added 
find_package(rclcpp REQUIRED) ## the ros 2 cpp execution lib
find_package(geometry_msgs REQUIRED)# message type i used
find_package(turtlesim REQUIRED)
 ## Providing a name so system produces a executable and 
 ##name it to be = cmd_vel_publisher
add_executable(cmd_vel_publisher src/cmd_vel_publisher.cpp)
add_executable(color_subscribed src/color_subscriber.cpp)

## adding all together , executable and its dependencies 
ament_target_dependencies(cmd_vel_publisher rclcpp geometry_msgs)
ament_target_dependencies(color_subscribed rclcpp turtlesim)

# Final link to make the system know what to execute 
install(TARGETS
cmd_vel_publisher # i just added this
color_subscribed # and this new subscriber's executbale
  DESTINATION lib/${PROJECT_NAME})
###


ament_package()
