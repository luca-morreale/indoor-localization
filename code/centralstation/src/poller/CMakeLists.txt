cmake_minimum_required(VERSION 2.8.3)
project(poller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  Measurement.msg
  MeasurementList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)



