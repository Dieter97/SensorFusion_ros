//
// Created by dieter on 03.04.19.
//

#include <ros/ros.h>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_fusion_msg/ObjectBoundingBox.h>
#include <sensor_fusion_msg/CameraObjects.h>

#include <fusion/fusion_objects/DarknetObject.h>

#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fusion/fusion_objects/FusedObject.h>
#include <visualization_msgs/Marker.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace sensor_fusion_msg;

// Main program variables

ros::Publisher pub, marker_pub;
DarknetObject * d;
float max = -1;

/**
 * Map function
 * @param value
 * @param low1
 * @param high1
 * @param low2
 * @param high2
 * @return
 */
float map(float value, float low1, float high1, float low2, float high2) {
    return (value - low1) * (high2 - low2) / (high1 - low1) + low2;
}

/**
 * Callback function, time synchronized callback function to process the data
 * @param image
 * @param cloud_msg
 * @param objects
 */
void callback(const ImageConstPtr &image, const PointCloud2ConstPtr &cloud_msg) {
    std::cout << "Cloud received" << std::endl;

    // First handle PointCloud Data
    auto *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *pclCloud);

    // Second handle image data
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    auto * fusedObjects = new std::vector<FusedObject*>;

    // Detect objects
    cv::imwrite("/tmp/tmp.png", cv_ptr->image);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<sensor_fusion_msg::ObjectBoundingBox> objects = d->detect_objects("/tmp/tmp.png",0.5f,0.5f,0.45f);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Time taken for object detection: " << elapsed.count() << std::endl;

    // Convert cameraObjects into fused objects
    for (const auto &bounding_box : objects) {
        auto boundingBox = new ObjectBoundingBox();
        boundingBox->x = bounding_box.x;
        boundingBox->y = bounding_box.y;
        boundingBox->w = bounding_box.w;
        boundingBox->h = bounding_box.h;
        boundingBox->Class = bounding_box.Class;
        boundingBox->probability = bounding_box.probability;
        fusedObjects->emplace_back(new FusedObject(boundingBox));
    }

    // Go through all pointcloud points to associate them with a bounding box
    for (auto it = pclCloud->begin(); it != pclCloud->end(); it++) {
        // First map the point to image coordinates
        auto * mappedPoint = new MappedPoint(*it, image->width, image->height, -3500, 0.27); //Scale is predetermined at -3500

        // Associate the points to a detected object
        for(const auto &fusedObject: *fusedObjects) {
            ObjectBoundingBox *box = fusedObject->cameraData;
            if (box->x-box->w/2 < mappedPoint->getPictureX() && mappedPoint->getPictureX() < box->x+box->w/2) {
                if (box->y-box->h/2 < mappedPoint->getPictureY() && mappedPoint->getPictureY() < box->y+box->h/2) {
                    fusedObject->addPoint(*mappedPoint);
                }
            }
        }
        /*
        // Draw the point
        if (mappedPoint->getDistance() > max) max = mappedPoint->getDistance();
        float color = map(mappedPoint->getDistance(), 0, max, 0, 255);
        int thickness = 6 - (int) map(mappedPoint->getDistance(), 0, max, 1, 5);

        cv::circle(cv_ptr->image, cv::Point((int) mappedPoint->getPictureX(), (int) mappedPoint->getPictureY()),
                   thickness,
                   cv::Scalar((int) color, 255 - (it->z * 10), 0), cv::FILLED, cv::LINE_8);
        */
    }

    // Draw objects
    for(const auto &fusedObject: *fusedObjects) {
        // Construct pointcloud from fused object points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(auto &point : *fusedObject->lidarPoints) {
            pcl::PointXYZ p = point.getPCLPoint();
            cloud->push_back(p);
        }

        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cloud, pcaCentroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
        // Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        // Final transform
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = bboxTransform[0];
        marker.pose.position.y = bboxTransform[1];
        marker.pose.position.z = bboxTransform[2];
        marker.pose.orientation.x = bboxQuaternion.x();
        marker.pose.orientation.y = bboxQuaternion.y();
        marker.pose.orientation.z = bboxQuaternion.z();
        marker.pose.orientation.w = bboxQuaternion.w();
        marker.scale.x = maxPoint.x-minPoint.x;
        marker.scale.y = maxPoint.y-minPoint.y;
        marker.scale.z = maxPoint.z-minPoint.z;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_pub.publish(marker);

        fusedObject->drawObject(cv_ptr);
    }


    pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_based_fuser");

    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_sub(nh, "/kitti/camera_gray_left/image_raw", 10); ///carla/ego_vehicle/camera/rgb/front/image_color
    message_filters::Subscriber<PointCloud2> info_sub(nh, "/lidar/detection/out/cropped", 10); ///lidar/detection/out/cropped
    //message_filters::Subscriber<CameraObjects> object_sub(nh, "/camera/detection/out", 1);
    typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::Image>("/camera/detection/out/image", 1);

    //End point to publish markers
    marker_pub = nh.advertise<visualization_msgs::Marker>("/fusion/bounding/out", 0);

    d = new DarknetObject("/home/dieter/darknet/cfg/yolov3.cfg","/home/dieter/darknet/data/yolov3.weights",0,"/home/dieter/darknet/cfg/coco.data");

    ros::spin();

    return 0;
}