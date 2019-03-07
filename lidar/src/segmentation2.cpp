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
    ///The smallest scale to use in the DoN filter.
    double scale1;

    ///The largest scale to use in the DoN filter.
    double scale2;

    ///The minimum DoN magnitude to threshold by
    double threshold;

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius;

    /// small scale
    scale1 = 10;
    /// large scale
    scale2 = 15;
    threshold = 0.1;   // threshold for DoN magnitude
    segradius = 2;   // threshold for radius segmentation

    // Load cloud in blob format
    pcl::PCLPointCloud2 blob;
    //pcl::io::loadPCDFile (infile.c_str (), blob);
    pcl_conversions::toPCL(*cloud_msg, blob);
    pcl::PointCloud<PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointXYZRGB>);
    pcl::fromPCLPointCloud2(blob, *cloud);

    std::cout << "Filtered Pointcloud: " << cloud->points.size() << " data points." << std::endl;


    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<PointXYZRGB>::Ptr tree;
    if (cloud->isOrganized()) {
        tree.reset(new pcl::search::OrganizedNeighbor<PointXYZRGB>());
    } else {
        tree.reset(new pcl::search::KdTree<PointXYZRGB>(false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud(cloud);

    if (scale1 >= scale2) {
        cerr << "Error: Large scale must be > small scale!" << endl;
        exit(EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<PointXYZRGB, PointXYZRGB> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);

    /**
     * NOTE: setting viewpoint is very important, so that we can ensure
     * normals are all pointed in the same direction!
     */
    ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max());

    // calculate normals with the small scale
    cout << "Calculating normals for scale..." << scale1 << endl;
    pcl::PointCloud<PointXYZRGB>::Ptr normals_small_scale(new pcl::PointCloud<PointXYZRGB>);

    ne.setRadiusSearch(scale1);
    ne.compute(*normals_small_scale);

    // calculate normals with the large scale
    cout << "Calculating normals for scale..." << scale2 << endl;
    pcl::PointCloud<PointXYZRGB>::Ptr normals_large_scale(new pcl::PointCloud<PointXYZRGB>);

    ne.setRadiusSearch(scale2);
    ne.compute(*normals_large_scale);

    // Create output cloud for DoN results
    PointCloud<PointXYZRGB>::Ptr doncloud(new pcl::PointCloud<PointXYZRGB>);
    copyPointCloud<PointXYZRGB, PointXYZRGB>(*cloud, *doncloud);

    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointXYZRGB, PointXYZRGB> don;
    don.setInputCloud(cloud);
    don.setNormalScaleLarge(normals_large_scale);
    don.setNormalScaleSmall(normals_small_scale);

    if (!don.initCompute()) {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature(*doncloud);

    // Save DoN features
    //pcl::PCDWriter writer;
    //writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);

    // Filter by magnitude
    cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

    // Build the condition for filtering
    pcl::ConditionOr<PointXYZRGB>::Ptr range_cond(
            new pcl::ConditionOr<PointXYZRGB>()
    );
    range_cond->addComparison(pcl::FieldComparison<PointXYZRGB>::ConstPtr(
            new pcl::FieldComparison<PointXYZRGB>("curvature", pcl::ComparisonOps::GT, threshold))
    );
    // Build the filter
    pcl::ConditionalRemoval<PointXYZRGB> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(doncloud);

    pcl::PointCloud<PointXYZRGB>::Ptr doncloud_filtered(new pcl::PointCloud<PointXYZRGB>);

    // Apply filter
    condrem.filter(*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->points.size() << " data points." << std::endl;

    //writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false);

    // Filter by magnitude
    cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

    pcl::search::KdTree<PointXYZRGB>::Ptr segtree(new pcl::search::KdTree<PointXYZRGB>);
    segtree->setInputCloud(doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointXYZRGB> ec;

    ec.setClusterTolerance(segradius);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(segtree);
    ec.setInputCloud(doncloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it, j++) {
        pcl::PointCloud<PointXYZRGB>::Ptr cloud_cluster_don(new pcl::PointCloud<PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            uint8_t r = 255, g = 0, b = 0;    // Example: Red color
            uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
            doncloud->points[*pit].rgb = *reinterpret_cast<float *>(&rgb);
            cloud_cluster_don->points.push_back(doncloud->points[*pit]);

        }

        cloud_cluster_don->width = int(cloud_cluster_don->points.size());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = false;


        pcl::PCLPointCloud2 point_cloud2;
        pcl::toPCLPointCloud2(*normals_large_scale, point_cloud2);
        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl_conversions::moveFromPCL(point_cloud2, output);
        // Publish the data
        pub.publish(*cloud_cluster_don);

        //Save cluster
        cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size() << " data points."
             << std::endl;
        //stringstream ss;
        //ss << "don_cluster_" << j << ".pcd";
        //writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);

    }
}

int main(int argc, char *argv[]) {
    // Initialize ROS
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}
