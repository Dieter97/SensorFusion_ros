//
// Created by dieter on 03.04.19.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_fusion_msg/ObjectBoundingBox.h>
#include <sensor_fusion_msg/CameraObjects.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace sensor_fusion_msg;

class MappedPoint {
private:
    pcl::PointXYZ point;
    float cameraPlane;
    int screenWidth;
    int screenHeight;
    int scale;
    float pictureX;
    float pictureY;
    float distance;

public:
    MappedPoint(pcl::PointXYZ point, int width, int height, int scale, float cameraPlane) {
        this->point = point;
        this->screenWidth = width;
        this->screenHeight = height;
        this->scale = scale;
        this->cameraPlane = cameraPlane;
        this->distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        this->mapPoint();
    }

    /*~MappedPoint() {
        delete(&this->point);
    }*/

    float mapPoint() {
        float *p;
        p = perspectiveMapping(point.x, point.y, point.z, cameraPlane);
        this->pictureX = ((*(p + 1) * scale) + this->screenWidth / 2);
        this->pictureY = ((*(p + 2) * scale) + this->screenHeight / 2);
    }

    float *perspectiveMapping(float x1, float y1, float z1, float cameraPlane) {
        float k = (cameraPlane - x1) / x1;
        static float result[3];
        result[0] = x1 + (k * x1);
        result[1] = y1 + (k * y1);
        result[2] = z1 + (k * z1);
        return result;
    }

    float getDistance() {
        return this->distance;
    }

    float map(float value, float low1, float high1, float low2, float high2) {
        return (value - low1) * (high2 - low2) / (high1 - low1) + low2;
    }

    float getPictureX() const {
        return pictureX;
    }

    void setPictureX(float pictureX) {
        MappedPoint::pictureX = pictureX;
    }

    float getPictureY() const {
        return pictureY;
    }

    void setPictureY(float pictureY) {
        MappedPoint::pictureY = pictureY;
    }
};

class FusedObject{
public:
    ObjectBoundingBox* cameraData;
    std::vector<MappedPoint> *lidarPoints;
    int r,g,b;

public:

    FusedObject(ObjectBoundingBox *cameraData) {
        this->cameraData = cameraData;
        this->lidarPoints = new std::vector<MappedPoint>;
        this->setRandomColor();
    }

    FusedObject(ObjectBoundingBox *cameraData, std::vector<MappedPoint> *points) {
        this->cameraData = cameraData;
        this->lidarPoints = points;
    }

    void setColor(int r, int g, int b) {
        this->r = r;
        this->g = g;
        this->b = b;
    }

    void setRandomColor(){
        this->r = std::rand() % 255;;
        this->g = std::rand() % 255;;
        this->b = std::rand() % 255;;
    }

    void addPoint(const MappedPoint& point) {
        this->lidarPoints->emplace_back(point);
    }

    void drawObject(const cv_bridge::CvImagePtr &imagePtr) {
        //Draw the bounding box first
        cv::Point pt1((int)(cameraData->x-cameraData->w/2), (int)(cameraData->y-cameraData->h/2));
        cv::Point pt2((int)(cameraData->x+cameraData->w/2), (int)(cameraData->y+cameraData->h/2));
        cv::rectangle(imagePtr->image, pt1, pt2, cv::Scalar(b,g,r));

        //Draw the lidar points
        for (auto it = lidarPoints->begin(); it != lidarPoints->end(); it++) {
            int thickness = 6 - (int) it->map(it->getDistance(), 0, 100, 1, 5);
            cv::circle(imagePtr->image, cv::Point((int) it->getPictureX(), (int) it->getPictureY()),
                       thickness,
                       cv::Scalar(b,g,r), cv::FILLED, cv::LINE_8);
        }
    }

};


// Main program variables

ros::Publisher pub;
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
void callback(const ImageConstPtr &image, const PointCloud2ConstPtr &cloud_msg, const CameraObjectsConstPtr &objects) {
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


    // Convert cameraObjects into fused objects
    for (const auto &bounding_box : objects->bounding_boxes) {
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
        auto * mappedPoint = new MappedPoint(*it, image->width, image->height, -3500, 0.2); //Scale is predetermined at -3500

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

    // Convert cameraObjects into fused objects
    for(const auto &fusedObject: *fusedObjects) {
        fusedObject->drawObject(cv_ptr);
    }


    pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_based_fuser");

    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_sub(nh, "/carla/ego_vehicle/camera/rgb/front/image_color", 10);
    message_filters::Subscriber<PointCloud2> info_sub(nh, "/lidar/detection/out/clusters", 10);
    message_filters::Subscriber<CameraObjects> object_sub(nh, "/camera/detection/out", 1);
    typedef sync_policies::ApproximateTime<Image, PointCloud2, CameraObjects> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub, object_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::Image>("/camera/detection/out/image", 1);

    ros::spin();

    return 0;
}