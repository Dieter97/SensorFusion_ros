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
find_package( OpenCV REQUIRED )

#For some reason msg have to be included like this
include_directories(../../devel/include)

include_directories(include)
include_directories(/home/dieter/darknet/include)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rosmy sensor_msgs std_msgs
#  DEPENDS system_lib
)

set(SOURCES
        src/fusion_objects/FusedObject.cpp
        src/fusion_objects/MappedPoint.cpp
        src/fusion_objects/DarknetObject.cpp
        include/fusion/fusion_objects/FusedObject.h
        include/fusion/fusion_objects/MappedPoint.h
        include/fusion/fusion_objects/DarknetObject.h)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lidar_camera_fusion src/lidar_camera_fusion.cpp ${SOURCES})
add_executable(lidar_object_fusion src/lidar_object_fusion.cpp ${SOURCES})
add_executable(cluster_camera_fusion src/cluster_camera_fusion.cpp ${SOURCES})
#add_dependencies(lidar_camera_fusion ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_camera_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(lidar_object_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(cluster_camera_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(lidar_camera_fusion sensor_fusion_msg_generate_messages_cpp)
add_dependencies(lidar_object_fusion sensor_fusion_msg_generate_messages_cpp)
add_dependencies(cluster_camera_fusion sensor_fusion_msg_generate_messages_cpp)