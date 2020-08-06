cmake_minimum_required(VERSION 2.8.3)
project(comm_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES 
  keypoint.msg
  keyframe.msg
  landmark.msg
  landmark_list.msg
  pose_update.msg
  pose_update_list.msg
  fused_pcl.msg
  pcl_transform.msg
)

generate_messages(
  DEPENDENCIES 
  nav_msgs 
  std_msgs 
  sensor_msgs 
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  nav_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
