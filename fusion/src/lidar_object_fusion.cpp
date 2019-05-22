//
// Created by dieter on 03.04.19.
//

#include <ros/ros.h>
#include <chrono>
#include <cstdio>
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
#include <visualization_msgs/MarkerArray.h>

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
DarknetObject *d;
int frame_id;
float max = -1;
double cameraPlane, scale;
std::string label_output_dir;

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

    auto *fusedObjects = new std::vector<FusedObject *>;

    // Detect objects
    cv::imwrite("/tmp/tmp.png", cv_ptr->image);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<sensor_fusion_msg::ObjectBoundingBox> objects = d->detect_objects("/tmp/tmp.png", 0.5f, 0.5f, 0.45f);

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
        auto *mappedPoint = new MappedPoint(*it, image->width, image->height, scale, cameraPlane); //Scale is predetermined at -3500

        // Associate the points to a detected object
        for (const auto &fusedObject: *fusedObjects) {
            ObjectBoundingBox *box = fusedObject->cameraData;
            if (box->x - box->w / 2 < mappedPoint->getPictureX() && mappedPoint->getPictureX() < box->x + box->w / 2) {
                if (box->y - box->h / 2 < mappedPoint->getPictureY() &&
                    mappedPoint->getPictureY() < box->y + box->h / 2) {
                    fusedObject->addPoint(*mappedPoint);
                }
            }
        }
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Time taken for object detection: " << elapsed.count() << std::endl;

    // Create output labels file
    char path[100];
    if(!label_output_dir.empty() && label_output_dir != " ") {
        std::ofstream file;
        sprintf(path, "%s/%06d.txt", label_output_dir.c_str(), frame_id);
        file.open(path);
        file.close();
    }

    // Draw objects & calculate a 3D bounding box for the object
    visualization_msgs::MarkerArray markers;
    for (const auto &fusedObject: *fusedObjects) {
        fusedObject->filterBiggestCluster(0.8);
        fusedObject->calculateBoundingBox();
        fusedObject->outputToLabelFile(path);
        //markers.markers.emplace_back(fusedObject->calculateBoundingBox());
        fusedObject->drawObject(cv_ptr);
    }

    // Publish output
    marker_pub.publish(markers);
    pub.publish(cv_ptr->toImageMsg());

    frame_id++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_based_fuser");

    //std::cout << "Label output dir (empty if no label output): ";
    //std::cin >> label_output_dir;

    ros::NodeHandle nh("~");
    std::string cameraInput;
    int bufferSize = 20;
    nh.getParam("cameraInput", cameraInput);
    nh.getParam("label", label_output_dir);
    nh.getParam("bufferSize", bufferSize);
    nh.getParam("cameraPlane", cameraPlane);
    nh.getParam("projectionScale", scale);

    message_filters::Subscriber<Image> image_sub(nh, cameraInput, bufferSize); ///carla/ego_vehicle/camera/rgb/front/image_color /kitti/camera_gray_left/image_raw
    message_filters::Subscriber<PointCloud2> info_sub(nh, "/lidar/detection/out/cropped", bufferSize); ///lidar/detection/out/cropped
    typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(bufferSize), image_sub, info_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::Image>("/camera/detection/out/image", 1);

    //End point to publish markers
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/fusion/bounding/out", 20);

    d = new DarknetObject("/home/dieter/darknet/cfg/yolov3.cfg", "/home/dieter/darknet/data/yolov3.weights", 0, "/home/dieter/darknet/cfg/coco.data");
    frame_id = 0;



    ros::spin();

    return 0;
}