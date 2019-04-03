//
// Created by dieter on 03.04.19.
//

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../../../devel/include/sensor_fusion_msg/ObjectBoundingBox.h"
#include "../../../devel/include/sensor_fusion_msg/CameraObjects.h"

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    std::cout << "Cloud received" << std::endl;
}


int main(int argc, char *argv[]) {
    // Initialize ROS
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/detection/out", 10,
                                                                 cloud_cb);
    // Spin
    ros::spin();

}