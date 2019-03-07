#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/organized.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> *objects;

void randomColorValue(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
    //objects->push_back(cloud);
    uint8_t r, g, b;
    r = std::rand() % 255;
    g = std::rand() % 255;
    b = std::rand() % 255;
    for (int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    ///The smallest scale to use in the DoN filter.
    double scale1 = 10;

    ///The largest scale to use in the DoN filter.
    double scale2 = 30;

    ///The minimum DoN magnitude to threshold by
    double threshold = 0.2;

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius = 2;

    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud2);
    pcl::fromPCLPointCloud2(*cloud2, *cloud);


    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
    if (cloud->isOrganized()) {
        tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>());
    } else {
        tree.reset(new pcl::search::KdTree<pcl::PointXYZRGB>(false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud(cloud);

    // Set the input pointcloud for the search tree
    tree->setInputCloud(cloud);

    if (scale1 >= scale2) {
        std::cerr << "Error: Large scale must be > small scale!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max());

    // calculate normals with the small scale
    std::cout << "Calculating normals for scale..." << scale1 << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    ne.setRadiusSearch(scale1);
    ne.compute(*normals_small_scale);

    // calculate normals with the large scale
    std::cout << "Calculating normals for scale..." << scale2 << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    ne.setRadiusSearch(scale2);
    ne.compute(*normals_large_scale);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(*cloud, *doncloud);

    std::cout << "Calculating DoN... " << std::endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> don;
    don.setInputCloud(cloud);
    don.setNormalScaleLarge(normals_large_scale);
    don.setNormalScaleSmall(normals_small_scale);
    if (!don.initCompute()) {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit(EXIT_FAILURE);
    }
    // Compute DoN
    don.computeFeature(*doncloud);

    // Filter by magnitude
    std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointXYZRGBNormal>::Ptr range_cond(
            new pcl::ConditionOr<pcl::PointXYZRGBNormal>()
    );
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZRGBNormal>("curvature", pcl::ComparisonOps::GT, threshold))
    );
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(doncloud);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Apply filter
    condrem.filter(*doncloud_filtered);
    doncloud = doncloud_filtered;

    // Filter by magnitude
    std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr segtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    segtree->setInputCloud(doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;

    ec.setClusterTolerance(segradius);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(segtree);
    ec.setInputCloud(doncloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it, j++) {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster_don(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster_don->points.push_back(doncloud->points[*pit]);
        }

        /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterOut(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t i = 0; i < cloud_cluster_don->points.size(); ++i) {
            const pcl::PointXYZRGB &mls_pt = cloud_cluster_don->points[i];
            pcl::PointXYZRGB pt(mls_pt.x, mls_pt.y, mls_pt.z);
            clusterOut->push_back(pt);
        }*/

        randomColorValue(cloud_cluster_don);
        cloud_cluster_don->width = static_cast<uint32_t>(int(cloud_cluster_don->points.size()));
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 out;
        pcl::toPCLPointCloud2(*cloud_cluster_don, out);
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

    objects = (new std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>());

    // Spin
    ros::spin();
}
