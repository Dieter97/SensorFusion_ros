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

ros::Publisher pub;

int scaleX = 20;
int scaleY = 30;
float camera_plane = 0.2;
float min=999,max=-1;


float * perspectiveMapping(float x1, float y1, float z1, float cameraPlane) {
    float k = (cameraPlane - x1)/x1;
    static float result[3];
    result[0] = x1 + (k * x1);
    result[1] = y1 + (k * y1);
    result[2] = z1 + (k * z1);
    return result;
}

float getDistance(float x1, float y1, float z1) {
    return std::sqrt(x1 * x1 + y1 * y1 + z1 * z1);
}

float map(float value, float low1, float high1, float low2, float high2) {
    return (value - low1) * (high2 - low2) / (high1 - low1) + low2;
}

void callback(const ImageConstPtr& image, const PointCloud2ConstPtr& cloud_msg) {
    std::cout << "Cloud received" << std::endl;

    // First handle PointCloud Data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *pclCloud);

    // Second handle image data
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Go through all pointcloud points to draw them on the image
    float camera_plane = 0.2;
    for(auto it = pclCloud->begin(); it!= pclCloud->end(); it++){
        /*float x_inverse =  (-1) / (it->x);
         = (int) (it->y * x_inverse * -camera_plane);  //(int)((((it->y*scale) - x_camera) * x_inverse) + x_camera)
         = (int) (it->z * x_inverse * -camera_plane);  //(int)((((it->z*scale) - y_camera) * x_inverse) + y_camera)*/
        float * p;
        p = perspectiveMapping(it->x,it->y,it->z,camera_plane);
        float new_X =  *(p+1);// * 0.5935f;  //(int)((((it->y*scale) - x_camera) * x_inverse) + x_camera)
        float new_Y =  *(p+2);// * 0.5935f;
        float distance = getDistance(it->x,it->y,it->z);
        if(distance > max) max = distance ;
        if(distance < min) min = distance ;
        float color = map(distance,0,max,0,255);
        int thickness = 6 - (int) map(distance,0,max,1,5);
        cv::circle(cv_ptr->image,cv::Point((int)((new_X*scaleX)+672),(int)((new_Y*scaleY)+188)),thickness,cv::Scalar( (int) color,255-(it->z*10), 0 ),cv::FILLED,cv::LINE_8);
    }

    pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    std::cout << "X_Scale: ";
    std::cin >> scaleX;
    std::cout << "Y_Scale: ";
    std::cin >> scaleY;
    std::cout << "Projection plane: ";
    std::cin >> camera_plane;

    message_filters::Subscriber<Image> image_sub(nh, "/carla/ego_vehicle/camera/rgb/front/image_color", 1);
    message_filters::Subscriber<PointCloud2> info_sub(nh, "/lidar/detection/out/clusters", 1);
    typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),image_sub, info_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::Image>("/camera/detection/out/image", 1);

    ros::spin();

    return 0;
}