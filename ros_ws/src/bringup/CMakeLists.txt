cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    ../../../config/zed_front.yaml
    ../../../config/localization_front.yaml
    ../../../config/detector_front.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
