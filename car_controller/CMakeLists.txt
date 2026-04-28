cmake_minimum_required(VERSION 3.8)
project(car_controller)

# ── C++ standard ─────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS2 dependencies ───────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ── pigpio system libraries ───────
find_library(PIGPIO_LIB pigpio
  HINTS /usr/local/lib
  REQUIRED
)
find_library(PIGPIOD_IF2_LIB pigpiod_if2
  HINTS /usr/local/lib
  REQUIRED
)

# ── Executable: car_controller_node ──────────
add_executable(car_controller_node
  src/car_controller_node.cpp
)
ament_target_dependencies(car_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  rcl_interfaces
)
target_link_libraries(car_controller_node
  ${PIGPIOD_IF2_LIB}   # pigpiod_if2: connects to the daemon via socket (needed in Docker)
)

target_include_directories(car_controller_node PRIVATE
  /usr/local/include
)

# ── Install ─────
install(TARGETS
  car_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  include
)

ament_package()
