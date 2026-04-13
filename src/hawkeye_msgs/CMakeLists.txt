cmake_minimum_required(VERSION 3.8)
project(hawkeye_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaggedImage.msg"
  DEPENDENCIES sensor_msgs geometry_msgs nav_msgs
)

ament_package()
