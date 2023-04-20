cmake_minimum_required(VERSION 3.5)
project(add_ring_field)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}
  src/add_ring_field_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
)
target_include_directories(${PROJECT_NAME}
  PUBLIC ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs PCL)
ament_package()
