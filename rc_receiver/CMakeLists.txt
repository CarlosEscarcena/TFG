cmake_minimum_required(VERSION 3.8)
project(rc_receiver)

# -- C++ standard -----
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -- ROS2 dependencies -------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# -- pigpio system libraries -------
find_library(PIGPIOD_IF2_LIB pigpiod_if2
  HINTS /usr/local/lib
  REQUIRED
)

# -- Executable: rc_reciever_node ----------
add_executable(rc_receiver_node
  src/rc_receiver_node.cpp
)
ament_target_dependencies(rc_receiver_node
  rclcpp
  geometry_msgs
  std_msgs
)
target_include_directories(rc_receiver_node PRIVATE
  include
  /usr/local/include
)
target_link_libraries(rc_receiver_node
  ${PIGPIOD_IF2_LIB}
)

# -- Install -----
install(TARGETS rc_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()