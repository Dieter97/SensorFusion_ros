cmake_minimum_required(VERSION 2.8.3)
project(fusion)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  cv_bridge
  sensor_msgs
  std_msgs
)
find_package(PCL  REQUIRED )

#For some reason msg have to be included like this
include_directories(../../devel/include)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rosmy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lidar_camera_fusion src/lidar_camera_fusion.cpp)
add_executable(lidar_object_fusion src/lidar_object_fusion.cpp)
#add_dependencies(lidar_camera_fusion ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_camera_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(lidar_object_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lidar_camera_fusion sensor_fusion_msg_generate_messages_cpp)
add_dependencies(lidar_object_fusion sensor_fusion_msg_generate_messages_cpp)