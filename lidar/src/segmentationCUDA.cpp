#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>
#include <time.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    int device = 0;

    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud2);
    pcl::fromPCLPointCloud2(*cloud2, *cloud);
/*
    pcl::gpu::setDevice (device);
    pcl::gpu::printShortCudaDeviceInfo (device);
*/
    /////////////////////////////////////////////
    /// GPU VERSION
    /////////////////////////////////////////////

    std::cout << "INFO: starting with the GPU version" << std::endl;

    clock_t tStart = clock();

    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(cloud->points);

    pcl::gpu::Octree::Ptr octree_device(new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();

    std::vector<pcl::PointIndices> cluster_indices_gpu;
    pcl::gpu::EuclideanClusterExtraction gec;
    gec.setClusterTolerance(0.02); // 2cm
    gec.setMinClusterSize(10);
    gec.setMaxClusterSize(25000);
    gec.setInput(cloud_device);
    gec.setSearchMethod(octree_device);
    gec.setHostCloud(cloud);
    gec.extract(cluster_indices_gpu);
    //octree_device.clear();

    printf("GPU Time taken: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);
    std::cout << "INFO: stopped with the GPU version" << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin();
         it != cluster_indices_gpu.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster_gpu->points.push_back(cloud->points[*pit]); //*
        cloud_cluster_gpu->width = cloud_cluster_gpu->points.size();
        cloud_cluster_gpu->height = 1;
        cloud_cluster_gpu->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size() << " data points."
                  << std::endl;
        /*std::stringstream ss;
        ss << "gpu_cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster_gpu, false); *///*
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
}


int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/front/point_cloud", 1,
                                                                 cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}