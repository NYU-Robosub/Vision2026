cmake_minimum_required(VERSION 3.8)
project(vision_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(fused_map_exporter src/fused_map_exporter.cpp)
ament_target_dependencies(fused_map_exporter rclcpp sensor_msgs std_srvs)

install(TARGETS fused_map_exporter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    ../../../config/zed_front.yaml
    ../../../config/zed_front_mapping_test.yaml
    ../../../config/zed_front_rtabmap.yaml
    ../../../config/rtabmap.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bringup test/test_bringup.py)
endif()

ament_package()
