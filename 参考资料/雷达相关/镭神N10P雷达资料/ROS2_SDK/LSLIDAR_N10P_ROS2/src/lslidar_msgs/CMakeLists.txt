cmake_minimum_required(VERSION 3.5)
project(lslidar_msgs)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(lslidar_msgs
  "msg/LslidarDifop.msg"
  "msg/LslidarPacket.msg"
  "msg/LslidarPoint.msg"
  "msg/LslidarScan.msg"
  "msg/LslidarSweep.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  )

ament_package()
