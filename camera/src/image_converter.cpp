#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::dnn::Net net;
    std::vector<std::string> classes;

public:
    ImageConverter() : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/carla/ego_vehicle/camera/rgb/front/image_color", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);

        // Load names of classes
        std::string classesFile = "/home/dieter/Ros/catkin_ws_sensorfusion/src/camera/src/yolo/coco.names";
        std::ifstream ifs(classesFile.c_str());
        std::string line;
        while (getline(ifs, line)) classes.push_back(line);

        // Give the configuration and weight files for the model
        std::string modelConfiguration = "/home/dieter/Ros/catkin_ws_sensorfusion/src/camera/src/yolo/yolov3.cfg";
        std::string modelWeights = "/home/dieter/Ros/catkin_ws_sensorfusion/src/camera/src/yolo/yolov3.weights";

        // Load the network
        net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
        net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat frame = cv_ptr->image;
        cv::Mat blob;
        if( frame.empty() ) {
            return;
        }
        cv::dnn::blobFromImage(frame, blob, 1/255.0, cvSize(416, 416), cv::Scalar(0,0,0), true, false);
        std::cout << "INFO: " << net.getLayerId("yolo") << std::endl;
        net.setInput(blob);

        std::vector<cv::Mat> outs;
        try{
            net.forward(outs, getOutputsNames(net));
        }catch(cv::Exception &e){
            std::cout << "ERROR: " << e.what() << std::endl;
        }


        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, frame);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }

    // Get the names of the output layers
    std::vector<std::string> getOutputsNames(const cv::dnn::Net& net)
    {
        static std::vector<std::string> names;
        if (names.empty())
        {
            //Get the indices of the output layers, i.e. the layers with unconnected outputs
            std::vector<int> outLayers = net.getUnconnectedOutLayers();

            //get the names of all the layers in the network
            std::vector<std::string> layersNames = net.getLayerNames();

            // Get the names of the output layers in names
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
                names[i] = layersNames[outLayers[i] - 1];
        }
        return names;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}