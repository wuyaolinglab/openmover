cmake_minimum_required(VERSION 3.5)
project(lslidar_driver_n10p)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(libpcap_LIBRARIES -lpcap)

#set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
#set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

#find_package(Boost REQUIRED COMPONENTS )
find_package(Boost REQUIRED thread)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common io)											   

include_directories(
		include
    ${PCL_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# Node
add_executable(lslidar_driver_node src/lslidar_driver_node.cc src/lslidar_driver.cc src/input.cc src/lsiosr.cpp)
target_link_libraries(lslidar_driver_node ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES} ${Boost_LIBRARIES} Boost::thread)
ament_target_dependencies(lslidar_driver_node rclcpp std_msgs lslidar_msgs sensor_msgs diagnostic_updater pcl_conversions)


install(DIRECTORY launch params rviz
		DESTINATION share/${PROJECT_NAME})

install(TARGETS
  lslidar_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp pluginlib lslidar_msgs sensor_msgs pcl_conversions)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

ament_package()
