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

void callback(const ImageConstPtr& image, const PointCloud2ConstPtr& cloud_msg)
{
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
    int x_camera = 0;
    int y_camera = 0;
    int scale = 1000;
    for(auto it = pclCloud->begin(); it!= pclCloud->end(); it++){
        float x_inverse =  (1 * scale) / (it->x*scale);
        int new_X = (int)((((it->y*scale) - x_camera) * x_inverse) + x_camera);  //(int) (it->y*scale*x_inverse)+xOffset;
        int new_Y = (int)((((it->z*scale) - y_camera) * x_inverse) + y_camera);  //(int)(it->z*scale*x_inverse)+yOffset);
        cv::circle(cv_ptr->image,cv::Point(new_X+672,-new_Y+188),1,cv::Scalar( 0, 0, 255 ),cv::FILLED,cv::LINE_8);
    }

    pub.publish(cv_ptr->toImageMsg());


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_sub(nh, "/carla/ego_vehicle/camera/rgb/front/image_color", 1);
    message_filters::Subscriber<PointCloud2> info_sub(nh, "output", 1);
    typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),image_sub, info_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::Image>("fused_output", 1);

    ros::spin();

    return 0;
}