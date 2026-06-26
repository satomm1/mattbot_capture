cmake_minimum_required(VERSION 3.0.2)
project(mattbot_capture)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  CaptureStartSession.srv
  CaptureStopSession.srv
  CaptureSaveFrame.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/capture_node.py
  scripts/capture_uploader.py
  scripts/capture_auto_trigger.py
  scripts/pose_logger.py
  scripts/pose_uploader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
