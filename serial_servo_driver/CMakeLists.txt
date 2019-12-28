cmake_minimum_required(VERSION 3.5)
project(serial_servo_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial_servo_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
ament_export_include_directories(include)

add_executable(serial_servo_driver src/serial_servo_driver.cpp)
ament_target_dependencies(serial_servo_driver rclcpp serial_servo_msgs)

add_executable(serial_servo_test src/serial_servo_test.cpp)
ament_target_dependencies(serial_servo_test rclcpp serial_servo_msgs)

install(TARGETS
  serial_servo_driver
  serial_servo_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
