cmake_minimum_required(VERSION 3.8)
project(vision_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    ../../../config/zed_front.yaml
    ../../../config/zed_front_mapping_test.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
