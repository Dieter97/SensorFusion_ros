cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion_msg)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  actionlib_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  pcl_msgs
  visualization_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
         ObjectBoundingBox.msg
         CameraObjects.msg
         LidarClusters.msg
         MappedPointMsg.msg
         FusedObjectsMsg.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
         actionlib_msgs
         geometry_msgs
         sensor_msgs
         std_msgs
         pcl_msgs
         sensor_fusion_msg
         visualization_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES sensor_fusion_msg
  CATKIN_DEPENDS actionlib_msgs geometry_msgs sensor_msgs std_msgs message_runtime visualization_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
 # ${catkin_INCLUDE_DIRS}
#)

