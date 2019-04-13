#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
int segTresh = 0;

uint8_t randomColorValue() {
    return std::rand() % 255;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr test(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*cloud_msg, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *test);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(test);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);


    float minX = 0;
    float maxX = 376;
    float minY = -672;
    float maxY = 672;
    float minZ = -100;
    float maxZ = 100;

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(test);
    boxFilter.filter(*filtered);

     // Convert to ROS data type
     sensor_msgs::PointCloud2 output;
     pcl::PCLPointCloud2 out;
     pcl::toPCLPointCloud2(*filtered, out);
     pcl_conversions::moveFromPCL(out, output);
     output.header.frame_id = cloud_msg->header.frame_id;
     // Publish the data
     pub.publish(output);
}


int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    /*std::cout << "Starting example Ros node! Threshold value: ";
    std::cin >> segTresh;*/

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/front/point_cloud", 1,
                                                                 cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}
