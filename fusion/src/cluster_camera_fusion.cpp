//
// Created by dieter on 25.04.19.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_fusion_msg/LidarClusters.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fusion/fusion_objects/FusedObject.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace sensor_fusion_msg;

// Main program variables
ros::Publisher pub;
float max = -1;

/**
 * Callback function, time synchronized callback function to process the data
 * @param image
 * @param cloud_msg
 * @param objects
 */
void callback(const ImageConstPtr &image, const LidarClustersConstPtr &clusters_msg) {
    std::cout << "Cloud received" << std::endl;

    std::vector<FusedObject*> clusters;

    // Transfrom cluster to FusedObject placeholders, these contain a pointcloud with points mapped to the image coordinate space
    for (const auto &cluster : clusters_msg->clusters) {
        auto *cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_conversions::toPCL(cluster, *cloud);
        pcl::fromPCLPointCloud2(*cloud, *pclCloud);
        auto * object = new FusedObject();
        for (auto it = pclCloud->begin(); it != pclCloud->end(); it++) {
            auto * mappedPoint = new MappedPoint(*it, image->width, image->height, -3500, 0.2); //Scale is predetermined at -3500
            object->addPoint(*mappedPoint);
        }

        clusters.emplace_back(object);

    }
/*
    // First handle PointCloud Data
    auto *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *pclCloud);
*/
    // Second handle image data
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    // Convert cameraObjects into fused objects
    for(const auto &fusedObject: clusters) {
        fusedObject->drawObject(cv_ptr);
    }

    pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "clustering_based_fuser");

    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_sub(nh, "/carla/ego_vehicle/camera/rgb/front/image_color", 10);
    message_filters::Subscriber<sensor_fusion_msg::LidarClusters> info_sub(nh, "/lidar/detection/out/clusters", 10);
    typedef sync_policies::ApproximateTime<Image, sensor_fusion_msg::LidarClusters> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::Image>("/camera/detection/out/image", 1);

    ros::spin();

    return 0;
}