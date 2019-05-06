/**
 * Difference of Normals Example for PCL Segmentation
 *
 */
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/features/don.h>
#include <pcl-1.9/pcl/filters/extract_indices.h>
#include <sensor_fusion_msg/LidarClusters.h>

// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>
#include <pcl/gpu/containers/initialization.h>

#include <time.h>
#include <algorithm>

using namespace pcl;
using namespace std;

ros::Publisher pub,pub2;
float segTresh = 0.1;


uint8_t randomColorValue() {
    return std::rand() % 255;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // Create object to store Pointcloud in PointXYZ
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    // convert the pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloud, *xyz_cloud_filtered);

    // Create object to store ground plane filtered pointcloud
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

    // Perform ransac planar filtration to remove ground plane
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
    seg1.setInputCloud (xyzCloudPtrFiltered);
    seg1.segment (*inliers, *coefficients);

    int i = 0;
    for(auto it = xyz_cloud_filtered->begin(); it!= xyzCloudPtrFiltered->end(); it++){
        if(!(std::find(inliers->indices.begin(), inliers->indices.end(), i) != inliers->indices.end())){
            xyz_cloud_ransac_filtered->points.push_back(*it);
        }
        i++;
    }

    // Perform cluster extraction on ground removed point cloud
    std::cout << "INFO: starting with the GPU version" << std::endl;

    clock_t tStart = clock();

    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(xyzCloudPtrRansacFiltered->points);

    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();

    std::vector<pcl::PointIndices> cluster_indices_gpu;
    pcl::gpu::EuclideanClusterExtraction gec;
    gec.setClusterTolerance (segTresh); // 2cm
    gec.setMinClusterSize (2);
    gec.setMaxClusterSize (25000);
    gec.setSearchMethod (octree_device);
    gec.setHostCloud(xyzCloudPtrRansacFiltered);
    gec.extract (cluster_indices_gpu);

    //  octree_device.clear();

    printf("GPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    std::cout << "INFO: stopped with the GPU version" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*xyzCloudPtrRansacFiltered,*rgb_cloud);

    std::vector<sensor_msgs::PointCloud2> clusters;
    int j = 0;
    // Process the clusters giving them a random color
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_gpu (new pcl::PointCloud<pcl::PointXYZRGB>);
        int r = randomColorValue();
        int g = randomColorValue();
        int b = randomColorValue();
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            rgb_cloud->points[*pit].r = r;
            rgb_cloud->points[*pit].g = g;
            rgb_cloud->points[*pit].b = b;
            cloud_cluster_gpu->points.push_back(rgb_cloud->points[*pit]);
        }

        cloud_cluster_gpu->width = cloud_cluster_gpu->points.size();
        cloud_cluster_gpu->height = 1;
        cloud_cluster_gpu->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size () << " data points." << std::endl;
        j++;

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 out;
        pcl::toPCLPointCloud2(*cloud_cluster_gpu, out);
        pcl_conversions::moveFromPCL(out, output);
        output.header.frame_id = cloud_msg->header.frame_id;
        output.header.stamp = cloud_msg->header.stamp;
        clusters.emplace_back(output);
        pub2.publish(output);
    }

    // Create lidarClusters message
    sensor_fusion_msg::LidarClusters clusterMsg;
    clusterMsg.header = cloud_msg->header;
    clusterMsg.clusters = clusters;
    pub.publish(clusterMsg);
}

int main(int argc, char *argv[]) {
    // Initialize ROS
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    std::cout << "Starting clustering Ros node! Threshold value: ";
    std::cin >> segTresh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar/detection/out/cropped", 10,
                                                                 cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_fusion_msg::LidarClusters>("/lidar/detection/out/clusters", 10);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("output", 10);

    // Spin
    ros::spin();
}
