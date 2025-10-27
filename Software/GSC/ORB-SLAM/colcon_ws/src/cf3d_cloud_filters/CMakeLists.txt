cmake_minimum_required(VERSION 3.8)
project(cf3d_cloud_filters)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

add_executable(temporal_pc_filter src/temporal_pc_filter.cpp)
ament_target_dependencies(temporal_pc_filter rclcpp sensor_msgs pcl_conversions)
target_link_libraries(temporal_pc_filter ${PCL_LIBRARIES})
target_include_directories(temporal_pc_filter PUBLIC ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(TARGETS temporal_pc_filter DESTINATION lib/${PROJECT_NAME})

ament_package()
