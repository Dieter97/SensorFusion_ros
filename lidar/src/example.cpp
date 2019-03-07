#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

uint8_t randomColorValue(){
    return std::rand() % 255;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *test);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(test);

    //for (pcl::PointXYZRGB point : test->points) {
    std::cout << "Size of pointcloud: " << (test->points).size() << std::endl;
        for(int i = 0 ; i < test->points.size();i++) {
            uint8_t r,g,b;
            if(i < test->points.size() / 2)
                 r = 0, g = 255, b = 0;    // Example: Red color
            else
                 r = 0, g = 0, b = 255;    // Example: Red color
            //uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            //test->points[i].rgb = (float)(rgb);
            test->points[i].r = randomColorValue();
            test->points[i].g = randomColorValue();
            test->points[i].b = randomColorValue();
            //std::cout << test->points[i].rgb << std::endl;
        }

    //}

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(test);
    sor.setLeafSize(1.0f, 1.0f, 1.0f);
    sor.filter(cloud_filtered);



    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 out;
    pcl::toPCLPointCloud2(cloud_filtered, out);
    pcl_conversions::moveFromPCL(out, output);

    // Publish the data
    pub.publish(output);
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

    // Spin
    ros::spin();
}
