#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.9/pcl/point_cloud.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/filters/crop_box.h>
#include <pcl-1.9/pcl/ModelCoefficients.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/io/pcd_io.h>
#include <pcl-1.9/pcl/filters/extract_indices.h>
#include <pcl-1.9/pcl/filters/voxel_grid.h>
#include <pcl-1.9/pcl/features/normal_3d.h>
#include <pcl-1.9/pcl/kdtree/kdtree.h>
#include <pcl-1.9/pcl/sample_consensus/method_types.h>
#include <pcl-1.9/pcl/sample_consensus/model_types.h>
#include <pcl-1.9/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.9/pcl/segmentation/extract_clusters.h>

// The GPU specific stuff here
#include <pcl-1.9/pcl/gpu/octree/octree.hpp>
#include <pcl-1.9/pcl/gpu/containers/device_array.hpp>
#include <pcl-1.9/pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl-1.9/pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>
#include <pcl-1.9/pcl/gpu/containers/initialization.h>
#include <time.h>

ros::Publisher pub;
int segTresh = 0;

uint8_t randomColorValue() {
    return std::rand() % 255;
}


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    int device = 0;

    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Create object to store Pointcloud in PointXYZRGB
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud (xyz_cloud);
    pcl::PointCloud<pcl::PointXYZRGB> *xyzrgb_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgbCloud (xyzrgb_cloud);

    // convert the pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloud, *xyzCloud);

    // Create object to store ground plane filtered pointclouid
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

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
    seg1.setInputCloud (xyzCloud);
    seg1.segment (*inliers, *coefficients);
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (xyzCloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*xyzCloudPtrRansacFiltered);


    pcl::gpu::setDevice (device);
    pcl::gpu::printShortCudaDeviceInfo (device);



/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    float minX = 0;
    float minY = -15;
    float minZ = -100;
    float maxX = 100;
    float maxY = 15;
    float maxZ = 100;

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*filtered);
*/
    /////////////////////////////////////////////
    /// GPU VERSION
    /////////////////////////////////////////////

    std::cout << "INFO: starting with the GPU version" << std::endl;

    clock_t tStart = clock();

    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(xyzCloudPtrRansacFiltered->points);

    pcl::gpu::Octree::Ptr octree_device(new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();

    std::vector<pcl::PointIndices> cluster_indices_gpu;
    pcl::gpu::EuclideanClusterExtraction gec;
    gec.setClusterTolerance(segTresh); // 2cm
    gec.setMinClusterSize(10);
    gec.setMaxClusterSize(25000);
    //gec.setInput(cloud_device);
    gec.setSearchMethod(octree_device);
    gec.setHostCloud(xyzCloudPtrRansacFiltered);
    gec.extract(cluster_indices_gpu);
    cloud_device.release();

    printf("GPU Time taken: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);
    std::cout << "INFO: stopped with the GPU version" << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin(); it != cluster_indices_gpu.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster_gpu->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]); //*
        cloud_cluster_gpu->width = cloud_cluster_gpu->points.size();
        cloud_cluster_gpu->height = 1;
        cloud_cluster_gpu->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size() << " data points."
                  << std::endl;
        j++;

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 out;
        pcl::toPCLPointCloud2(*cloud_cluster_gpu, out);
        pcl_conversions::moveFromPCL(out, output);
        output.header.frame_id = cloud_msg->header.frame_id;
        // Publish the data
        pub.publish(output);
    }

    printf("GPU Time taken: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);
    std::cout << "INFO: stopped with the GPU version" << std::endl;

}


int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    std::cout << "Starting example Ros node! Threshold value: ";
    std::cin >> segTresh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/front/point_cloud", 1,
                                                                 cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}