cmake_minimum_required(VERSION 3.0.2)
project(ros_lidar_examples)

## Используем Python
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package()

## Установка Python-скриптов
catkin_install_python(PROGRAMS
  scripts/fake_scan_publisher.py
  scripts/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
