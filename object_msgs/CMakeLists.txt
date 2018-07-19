cmake_minimum_required(VERSION 2.8.3)
project(object_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES
  Object.msg
  Objects.msg
  ObjectInBox.msg
  ObjectsInBoxes.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs   
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS  sensor_msgs std_msgs message_runtime 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

