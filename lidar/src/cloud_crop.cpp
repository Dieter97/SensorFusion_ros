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

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (test);
    vg.setLeafSize (0.50f, 0.50f, 0.50f);
    vg.filter (*cloud_downsample);
    std::cout << "PointCloud after filtering has: " << cloud_downsample->points.size ()  << " data points." << std::endl;

    float minX = 0;
    float maxX = 376;
    float minY = -672;
    float maxY = 672;
    float minZ = -100;
    float maxZ = 100;

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud_downsample);
    boxFilter.filter(*filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr groundRemoved(new pcl::PointCloud<pcl::PointXYZ>);

    // perform ransac planar filtration to remove ground plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg1;
    // Optional
    seg1.setOptimizeCoefficients (true);
    // Mandatory
    seg1.setModelType (pcl::SACMODEL_PLANE);
    seg1.setMethodType (pcl::SAC_RANSAC);
    seg1.setDistanceThreshold (0.4);
    seg1.setInputCloud (filtered);
    seg1.segment (*inliers, *coefficients);

    int i = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = filtered->begin(); it!= filtered->end(); it++){
        if(!(std::find(inliers->indices.begin(), inliers->indices.end(), i) != inliers->indices.end())){
            groundRemoved->points.push_back(*it);
        }
        i++;
    }


     // Convert to ROS data type
     sensor_msgs::PointCloud2 output;
     pcl::PCLPointCloud2 out;
     pcl::toPCLPointCloud2(*groundRemoved, out);
     pcl_conversions::moveFromPCL(out, output);
     output.header.frame_id = cloud_msg->header.frame_id;
     output.header.stamp = cloud_msg->header.stamp;
     // Publish the data
     pub.publish(output);
}


int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "cloud_crop");
    ros::NodeHandle nh;

    /*std::cout << "Starting example Ros node! Threshold value: ";
    std::cin >> segTresh;*/

    std::string input;
    nh.getParam("input", input);

    // Create a ROS subscriber for the input point cloud /carla/ego_vehicle/lidar/front/point_cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar/detection/out/cropped", 1);

    // Spin
    ros::spin();
}
