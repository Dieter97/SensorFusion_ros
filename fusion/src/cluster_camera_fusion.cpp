//
// Created by dieter on 25.04.19.
//

#include <ros/ros.h>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_fusion_msg/LidarClusters.h>

#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fusion/fusion_objects/FusedObject.h>
#include <fusion/fusion_objects/DarknetObject.h>

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
DarknetObject * d;
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

    // Transform cluster to FusedObject placeholders, these contain a pointcloud with points mapped to the image coordinate space
    for (const auto &cluster : clusters_msg->clusters) {
        auto *cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_conversions::toPCL(cluster, *cloud);
        pcl::fromPCLPointCloud2(*cloud, *pclCloud);

        //Create fused object with all lidar points mapped to image coordinates and propose a bounding box
        auto * object = new FusedObject();
        object->cameraData = new ObjectBoundingBox();
        int minX=image->width,maxX=0,minY=image->height,maxY=0;
        for (auto it = pclCloud->begin(); it != pclCloud->end(); it++) {
            auto * mappedPoint = new MappedPoint(*it, image->width, image->height, -3500, 0.2); //Scale is predetermined at -3500
            object->addPoint(*mappedPoint);

            // Propose a 2D bouding box for the cluster
            if(minX > mappedPoint->getPictureX()) minX = (int) mappedPoint->getPictureX();
            if(maxX < mappedPoint->getPictureX()) maxX = (int) mappedPoint->getPictureX();
            if(minY > mappedPoint->getPictureY()) minY = (int) mappedPoint->getPictureY();
            if(maxY < mappedPoint->getPictureY()) maxY = (int) mappedPoint->getPictureY();
        }

        // Update the object' bounding box and add an offset
        object->cameraData->x = minX + (int)((maxX-minX)/2);
        object->cameraData->y = minY + (int)((maxY-minY)/2);
        object->cameraData->w = (int)((maxX-minX)) + (int) (object->lidarPoints->size() / 2);
        object->cameraData->h = (int)((maxY-minY)) + (int) (object->lidarPoints->size() / 2);
        object->setRandomColor();
        clusters.emplace_back(object);
    }

    // Second handle image data
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Loop through all clusters, perform object detection and draw

    for(const auto &fusedObject: clusters) {
        cv::Point pt1((int)(fusedObject->cameraData->x-fusedObject->cameraData->w/2), (int)(fusedObject->cameraData->y-fusedObject->cameraData->h/2));
        cv::Point pt2((int)(fusedObject->cameraData->x+fusedObject->cameraData->w/2), (int)(fusedObject->cameraData->y+fusedObject->cameraData->h/2));
        cv::Rect rect(pt1,pt2);
        cv::Mat miniMat;
        if(rect.x > 0 && rect.x < image->width && rect.x+rect.width > 0 && rect.x+rect.width < image->width){
            if(rect.y > 0 && rect.y < image->height && rect.y+rect.height > 0 && rect.y+rect.height < image->height){
                miniMat = cv_ptr->image(rect);
                cv::imwrite("/tmp/tmp.png", miniMat);
                auto start = std::chrono::high_resolution_clock::now();
                std::vector<sensor_fusion_msg::ObjectBoundingBox> objects = d->detect_objects("/tmp/tmp.png",0.5f,0.5f,0.45f);
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish - start;
                std::cout << "Time taken for object detection: " << elapsed.count() << std::endl;
                if(!objects.empty()){
                    std::cout << "Detected class" << objects[0].Class << std::endl;
                    fusedObject->cameraData->Class = objects[0].Class;
                    fusedObject->cameraData->probability = objects[0].probability;
                    fusedObject->drawObject(cv_ptr);
                }
            }
        }

    }
/*
    cv::imwrite("/tmp/tmp.png", cv_ptr->image);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<sensor_fusion_msg::ObjectBoundingBox> objects = d->detect_objects("/tmp/tmp.png",0.5f,0.5f,0.45f);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Time taken for object detection: " << elapsed.count() << std::endl;
*/

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

    d = new DarknetObject("/home/dieter/darknet/cfg/yolov3.cfg","/home/dieter/darknet/data/yolov3.weights",0,"/home/dieter/darknet/cfg/coco.data");

    ros::spin();

    return 0;
}