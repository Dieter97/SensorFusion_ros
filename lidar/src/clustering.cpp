#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.9/pcl/point_cloud.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/filters/crop_box.h>
#include <pcl-1.9/pcl/sample_consensus/method_types.h>
#include <pcl-1.9/pcl/sample_consensus/model_types.h>
#include <pcl-1.9/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.9/pcl/filters/voxel_grid.h>
#include <pcl-1.9/pcl/filters/extract_indices.h>
#include <pcl-1.9/pcl/kdtree/kdtree.h>
#include <pcl-1.9/pcl/segmentation/extract_clusters.h>
#include <sensor_fusion_msg/LidarClusters.h>


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

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr test(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform voxel grid downsampling filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (*cloud_filtered);

    // Create object to store Pointcloud in PointXYZRGB
    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

    // convert the pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloud_filtered, *xyzCloudPtrFiltered);

    // Create object to store ground plane filtered pointclouid
    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

    // perform ransac planar filtration to remove ground plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
    // Optional
    seg1.setOptimizeCoefficients (true);
    // Mandatory
    seg1.setModelType (pcl::SACMODEL_PLANE);
    seg1.setMethodType (pcl::SAC_RANSAC);
    seg1.setDistanceThreshold (0.4);
    seg1.setInputCloud (xyzCloudPtrFiltered);
    seg1.segment (*inliers, *coefficients);
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (xyzCloudPtrFiltered);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*xyzCloudPtrRansacFiltered);

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (xyzCloudPtrRansacFiltered);


    // create the extraction object for the clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // specify euclidean cluster parameters
    ec.setClusterTolerance (segTresh); //
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (40);
    ec.setSearchMethod (tree);
    ec.setInputCloud (xyzCloudPtrRansacFiltered);
    // extract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ec.extract (cluster_indices);

    int j = 0;
    std::vector<sensor_msgs::PointCloud2> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_gpu(new pcl::PointCloud<pcl::PointXYZRGB>);
        int r = randomColorValue();
        int g = randomColorValue();
        int b = randomColorValue();
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            xyzCloudPtrRansacFiltered->points[*pit].r = r;
            xyzCloudPtrRansacFiltered->points[*pit].g = g;
            xyzCloudPtrRansacFiltered->points[*pit].b = b;
            cloud_cluster_gpu->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
        }

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
        clusters.emplace_back(output);

    }

    // Create lidarClusters message
    sensor_fusion_msg::LidarClusters clusterMsg;
    clusterMsg.header = cloud_msg->header;
    clusterMsg.clusters = clusters;
    pub.publish(clusterMsg);


    //pcl::fromPCLPointCloud2(*cloud, *test);

    /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(test);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


    float minX = 0;
    float minY = -15;
    float minZ = -100;
    float maxX = 100;
    float maxY = 15;
    float maxZ = 100;

    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(test);
    boxFilter.filter(*filtered);*/

    /*
    //for (pcl::PointXYZRGB point : test->points) {
    std::cout << "Size of pointcloud: " << (test->points).size() << std::endl;
    for (int i = 0; i < test->points.size(); i++) {
        uint8_t r, g, b;
        if (i < test->points.size() / 2)
            r = 0, g = 255, b = 0;    // Example: Red color
        else
            r = 0, g = 0, b = 255;    // Example: Red color
        //uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        //test->points[i].rgb = (float)(rgb);
        test->points[i].r = randomColorValue();
        test->points[i].g = randomColorValue();
        test->points[i].b = randomColorValue();
        //std::cout << test->points[i].rgb << std::endl;
    }*/

    //}
    /*
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(test);
    sor.setLeafSize(1.0f, 1.0f, 1.0f);
    sor.filter(cloud_filtered);
    */


   /* // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 out;
    pcl::toPCLPointCloud2(*xyzCloudPtrRansacFiltered, out);
    pcl_conversions::moveFromPCL(out, output);

    // Publish the data
    pub.publish(output);*/
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
    pub = nh.advertise<sensor_fusion_msg::LidarClusters>("/lidar/detection/out/clusters", 1);

    // Spin
    ros::spin();
}
