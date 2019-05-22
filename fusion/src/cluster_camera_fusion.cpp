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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>


using namespace sensor_msgs;
using namespace message_filters;
using namespace sensor_fusion_msg;

// Main program variables
ros::Publisher pub, marker_pub;
DarknetObject * d;
float max = -1;
int frame_id;
double cameraPlane, scale;
std::string label_output_dir;

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
        for (auto it = pclCloud->begin(); it != pclCloud->end(); it++) {
            auto * mappedPoint = new MappedPoint(*it, image->width, image->height, scale, cameraPlane); //Scale is predetermined at -3500 0.27 is the distance between the camera and lidar
            object->addPoint(*mappedPoint);
         }
        object->filterBiggestCluster(0.8); // Cluster again using a smaller threshold

        int minX=image->width,maxX=0,minY=image->height,maxY=0;
        for (auto it = object->lidarPoints->begin(); it != object->lidarPoints->end(); it++) {
            // Propose a 2D bounding box for the cluster
            if(minX > it->getPictureX()) minX = (int) it->getPictureX();
            if(maxX < it->getPictureX()) maxX = (int) it->getPictureX();
            if(minY > it->getPictureY()) minY = (int) it->getPictureY();
            if(maxY < it->getPictureY()) maxY = (int) it->getPictureY();
        }

        // Update the object' bounding box and add an offset
        object->cameraData->x = minX + (int)((maxX-minX)/2);
        object->cameraData->y = minY + (int)((maxY-minY)/2);
        object->cameraData->w = (int) ((maxX - minX)) + 15;
        object->cameraData->h = (int) ((maxY - minY)) + 15;
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

    // Create output labels file
    char path[100];
    if(!label_output_dir.empty() && label_output_dir != " ") {
        std::ofstream file;
        sprintf(path, "%s/%06d.txt", label_output_dir.c_str(), frame_id);
        file.open(path);
        file.close();
    }

    auto start = std::chrono::high_resolution_clock::now();

    // Loop through all clusters, perform object detection and draw
    visualization_msgs::MarkerArray markers;
    for(const auto &fusedObject: clusters) {
        cv::Point pt1((int)(fusedObject->cameraData->x-fusedObject->cameraData->w/2), (int)(fusedObject->cameraData->y-fusedObject->cameraData->h/2));
        cv::Point pt2((int)(fusedObject->cameraData->x+fusedObject->cameraData->w/2), (int)(fusedObject->cameraData->y+fusedObject->cameraData->h/2));
        cv::Rect rect(pt1,pt2);
        cv::Mat miniMat;
        if(rect.x > 0 && rect.x < image->width && rect.x+rect.width > 0 && rect.x+rect.width < image->width){
            if(rect.y > 0 && rect.y < image->height && rect.y+rect.height > 0 && rect.y+rect.height < image->height){
                if ((rect.width * rect.height) > 800) { //Require the bounding box to have a minimum area
                    miniMat = cv_ptr->image(rect);
                    cv::imwrite("/tmp/tmp.png", miniMat);
                    // Detect objects in image based on bounding box
                    std::vector<sensor_fusion_msg::ObjectBoundingBox> objects = d->detect_objects("/tmp/tmp.png", 0.5f,0.5f, 0.45f);
                    if (!objects.empty()) {
                        std::cout << "Detected class" << objects[0].Class << std::endl;
                        fusedObject->cameraData->Class = objects[0].Class;
                        fusedObject->cameraData->probability = objects[0].probability;
                        fusedObject->cameraData->x = pt1.x + objects[0].x;
                        fusedObject->cameraData->y = pt1.y + objects[0].y;
                        fusedObject->cameraData->w = objects[0].w;
                        fusedObject->cameraData->h = objects[0].h;
                        fusedObject->filterPointCloudOutsideBB();
                        fusedObject->calculateBoundingBox();
                        //markers.markers.emplace_back(fusedObject->calculateBoundingBox());
                        fusedObject->outputToLabelFile(path);
                        fusedObject->drawObject(cv_ptr);
                    }
                }

            }
        }

    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Time taken for object detection: " << elapsed.count() << std::endl;

    // Publish outputs
    marker_pub.publish(markers);
    pub.publish(cv_ptr->toImageMsg());

    frame_id++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "clustering_based_fuser");
    ros::NodeHandle nh("~");
    std::string cameraInput;
    int bufferSize = 20;
    nh.getParam("label", label_output_dir);
    nh.getParam("cameraInput", cameraInput);
    nh.getParam("bufferSize", bufferSize);
    nh.getParam("cameraPlane", cameraPlane);
    nh.getParam("projectionScale", scale);

    message_filters::Subscriber<Image> image_sub(nh, cameraInput, bufferSize); ///kitti/camera_color_left/image_raw
    message_filters::Subscriber<sensor_fusion_msg::LidarClusters> info_sub(nh, "/lidar/detection/out/clusters", bufferSize);
    typedef sync_policies::ApproximateTime<Image, sensor_fusion_msg::LidarClusters> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(bufferSize), image_sub, info_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::Image>("/camera/detection/out/image", 1);

    //End point to publish markers
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/fusion/bounding/out", 20);

    d = new DarknetObject("/home/dieter/darknet/cfg/yolov2-tiny.cfg", "/home/dieter/darknet/data/yolov2-tiny.weights", 0, "/home/dieter/darknet/cfg/coco.data");
    frame_id = 0;


    ros::spin();

    return 0;
}