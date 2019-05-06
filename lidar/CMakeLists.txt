cmake_minimum_required(VERSION 2.8.3)
project(lidar)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
)
find_package(PCL  REQUIRED )

# Include message files
include_directories(../../devel/include)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(clustering src/clustering.cpp)
add_executable(segmentation src/segmentation.cpp)
add_executable(clusteringCUDA src/clusteringCUDA.cpp)
add_executable(cloud_crop src/cloud_crop.cpp)
add_executable(donSegmentationCUDA src/donSegmentationCUDA.cpp)
target_link_libraries(clustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(clusteringCUDA ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cloud_crop ${catkin_LIBRARIES}  ${PCL_LIBRARIES})
target_link_libraries(donSegmentationCUDA ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

