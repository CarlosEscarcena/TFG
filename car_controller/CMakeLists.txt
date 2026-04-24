cmake_minimum_required(VERSION 3.8)
project(car_controller)

# ── Estándares C++ ────────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencias ROS2 ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ── Librería pigpio (en el sistema) ───────────────────────────────────────────
find_library(PIGPIO_LIB pigpio
  HINTS /usr/local/lib
  REQUIRED
)
find_library(PIGPIOD_IF2_LIB pigpiod_if2
  HINTS /usr/local/lib
  REQUIRED
)
# ── Ejecutable: car_controller_node ──────────────────────────────────────────
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
  ${PIGPIOD_IF2_LIB}   # pigpiod_if2: se conecta al daemon via socket (recomendado en Docker)
)

target_include_directories(car_controller_node PRIVATE
  /usr/local/include
)

# ── Ejecutable: cmd_vel_publisher ────────────────────────────────────────────
add_executable(cmd_vel_publisher
  src/cmd_vel_publisher.cpp
)
ament_target_dependencies(cmd_vel_publisher
  rclcpp
  geometry_msgs
)

# ── Ejecutable: keyboard_teleop ──────────────────────────────────────────────
add_executable(keyboard_teleop
  src/keyboard_teleop.cpp
)
ament_target_dependencies(keyboard_teleop
  rclcpp
  geometry_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  car_controller_node
  cmd_vel_publisher
  keyboard_teleop
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

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
