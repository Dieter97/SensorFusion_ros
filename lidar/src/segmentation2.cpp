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
#include "pcl_ros/point_cloud.h"
#include <pcl/features/don.h>

using namespace pcl;
using namespace std;

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud2);
    pcl::fromPCLPointCloud2(*cloud2, *cloud);


    /////////////////////////////////////////////
    /// CPU VERSION
    /////////////////////////////////////////////

    std::cout << "INFO: PointCloud_filtered still has " << cloud->points.size() << " Points " << std::endl;
    clock_t tStart = clock();
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    printf("CPU Time taken: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
                  << std::endl;
        std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); ///
        j++;

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 out;
        pcl::toPCLPointCloud2(*cloud_cluster, out);
        pcl_conversions::moveFromPCL(out, output);
        output.header.frame_id = cloud_msg->header.frame_id;
        // Publish the data
        pub.publish(output);

    }
}

int main(int argc, char *argv[]) {
    // Initialize ROS
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/front/point_cloud", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}
