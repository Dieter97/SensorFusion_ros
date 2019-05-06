
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.9/pcl/point_cloud.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/io/pcd_io.h>
#include <pcl-1.9/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.9/pcl/common/point_operators.h>
#include <pcl-1.9/pcl/common/io.h>
#include <pcl-1.8/pcl/filters/conditional_removal.h>
#include <pcl-1.9/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.9/pcl/segmentation/extract_clusters.h>

#include <pcl-1.9/pcl/gpu/features/features.hpp>
#include <pcl-1.9/pcl/gpu/octree/octree.hpp>

#include <pcl-1.9/pcl/features/don.h>

using namespace pcl;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal PointNT;
typedef pcl::PointNormal PointOutT;

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    ///The smallest scale to use in the DoN filter.
    double scale1 = 0.2;

    ///The smallest scale to use in the DoN filter.
    double scale2 = 2;

    ///The file to read from.
    string infile;

    ///The file to output to.
    string outfile;

    ///The minimum DoN magnitude to threshold by
    double threshold = 0.25;

    ///The euclidian cluster distance to use
    double segradius = 0.2;


    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cout << "Converting point cloud...";
    PointCloud<pcl::PointXYZ>::Ptr xyzcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *xyzcloud);
    copyPointCloud<pcl::PointXYZ, PointT>(*xyzcloud, *cloud);
    cout << "done." << endl;

    cout << "Uploading point cloud to GPU ..." << endl;
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(xyzcloud->points);

    // Compute normals using both small and large scales at each point
    // TODO: Use IntegralImageNormalEstimation for organized data
    pcl::gpu::NormalEstimation ne;
    ne.setInputCloud(cloud_device);

    /**
     * NOTE: setting viewpoint is very important, so that we can ensure
     * normals are all pointed in the same direction!
     */
    ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max());

    if (scale1 >= scale2) {
        cerr << "Error: Large scale must be > small scale!" << endl;
        exit(EXIT_FAILURE);
    }

    //maximum answers for search radius
    //NOTE: lower this if you are running out of GPU memory
    const int max_answers = 500;
    //buffer for results
    cout << "Creating GPU NormalEstimation output dev..." << endl;
    pcl::gpu::Feature::Normals result_device(cloud_device.size());

    //the normals calculated with the small scale
    cout << "Calculating normals for scale..." << scale1 << endl;
    ne.setRadiusSearch(scale1, max_answers);
    ne.compute(result_device);

    //the normals calculated with the small scale
    cout << "Downloading results from GPU..." << endl;
    std::vector<PointXYZ> normals_small_scale_vec(result_device.size());
    result_device.download(normals_small_scale_vec);

    pcl::PointCloud<PointNT>::Ptr normals_small_scale(new pcl::PointCloud<PointNT>);
    for (std::vector<PointXYZ>::iterator resultpt = normals_small_scale_vec.begin();
         resultpt != normals_small_scale_vec.end(); resultpt++) {
        normals_small_scale->push_back(Normal(resultpt->x, resultpt->y, resultpt->z));
    }

    cout << "Calculating normals for scale..." << scale2 << endl;
    //the normals calculated with the large scale
    ne.setRadiusSearch(scale2, max_answers);
    ne.compute(result_device);

    cout << "Downloading results from GPU..." << endl;
    std::vector<PointXYZ> normals_large_scale_vec(result_device.size());
    result_device.download(normals_large_scale_vec);

    pcl::PointCloud<PointNT>::Ptr normals_large_scale(new pcl::PointCloud<PointNT>);
    for (std::vector<PointXYZ>::iterator resultpt = normals_large_scale_vec.begin();
         resultpt != normals_large_scale_vec.end(); resultpt++) {
        normals_large_scale->push_back(Normal(resultpt->x, resultpt->y, resultpt->z));
    }

    // Create output cloud for DoN results
    PointCloud<PointOutT>::Ptr doncloud(new pcl::PointCloud<PointOutT>);
    pcl::fromROSMsg(*cloud_msg, *xyzcloud);
    copyPointCloud<pcl::PointXYZ, PointOutT>(*xyzcloud, *doncloud);

    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PointT, PointNT, PointOutT> don;
    don.setInputCloud(cloud);
    don.setNormalScaleLarge(normals_large_scale);
    don.setNormalScaleSmall(normals_small_scale);

    if (!don.initCompute()) {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit(EXIT_FAILURE);
    }

    //Compute DoN
    don.computeFeature(*doncloud);

    cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

    // build the condition
    pcl::ConditionOr<PointOutT>::Ptr range_cond(new
                                                        pcl::ConditionOr<PointOutT>());
    range_cond->addComparison(pcl::FieldComparison<PointOutT>::ConstPtr(new pcl::FieldComparison<PointOutT>(
            "curvature", pcl::ComparisonOps::GT, threshold)));
    // build the filter
    pcl::ConditionalRemoval<PointOutT> condrem(range_cond);
    condrem.setInputCloud(doncloud);

    pcl::PointCloud<PointOutT>::Ptr doncloud_filtered(new pcl::PointCloud<PointOutT>);

    // apply filter
    condrem.filter(*doncloud_filtered);

    doncloud = doncloud_filtered;


    cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

    pcl::search::KdTree<PointOutT>::Ptr segtree(new pcl::search::KdTree<PointOutT>);
    segtree->setInputCloud(doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointOutT> ec;

    ec.setClusterTolerance(segradius);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(segtree);
    ec.setInputCloud(doncloud);
    ec.extract(cluster_indices);

    pcl::PCDWriter writer;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        pcl::PointCloud<PointOutT>::Ptr cloud_cluster(new pcl::PointCloud<PointOutT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_cluster->points.push_back(doncloud->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
                  << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<PointOutT>(ss.str(), *cloud_cluster, false); //*

        j++;
        // Save filtered output
        sensor_msgs::PointCloud2 outblob;
        pcl::toROSMsg(*doncloud, outblob);
        pub.publish(outblob);
    }
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 1,
                                                                 cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}