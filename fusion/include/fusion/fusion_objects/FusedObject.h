//
// Created by dieter on 25.04.19.
//

#ifndef PROJECT_FUSEDOBJECT_H
#define PROJECT_FUSEDOBJECT_H

#include <sensor_fusion_msg/ObjectBoundingBox.h>
#include <sensor_fusion_msg/CameraObjects.h>
#include <sensor_fusion_msg/FusedObjectsMsg.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include "MappedPoint.h"
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl-1.9/pcl/filters/extract_indices.h>
#include <pcl-1.9/pcl/kdtree/kdtree.h>
#include <pcl-1.9/pcl/segmentation/extract_clusters.h>

#include <time.h>
#include <algorithm>
#include <fstream>


using namespace sensor_fusion_msg;

class FusedObject {
public:
    ObjectBoundingBox* cameraData;
    visualization_msgs::Marker* bbox;
    std::vector<MappedPoint> *lidarPoints;
    int r,g,b;

public:
    FusedObject();

    explicit FusedObject(ObjectBoundingBox *cameraData);

    FusedObject(ObjectBoundingBox *cameraData, std::vector<MappedPoint> *points);

    void setColor(int r, int g, int b);

    void setRandomColor();

    void setBbox(visualization_msgs::Marker *bbox);

    void addPoint(const MappedPoint& point);

    void drawObject(const cv_bridge::CvImagePtr &imagePtr);

    void filterBiggestCluster(float tolerance);

    visualization_msgs::Marker* calculateBoundingBox();

    void outputToLabelFile(char *fileLocation);

    void filterPointCloudOutsideBB();

    void toMsg(sensor_fusion_msg::FusedObjectsMsgPtr message);
};


#endif //PROJECT_FUSEDOBJECT_H
