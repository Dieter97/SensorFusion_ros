//
// Created by dieter on 25.04.19.
//

#ifndef PROJECT_FUSEDOBJECT_H
#define PROJECT_FUSEDOBJECT_H

#include <sensor_fusion_msg/ObjectBoundingBox.h>
#include <sensor_fusion_msg/CameraObjects.h>
#include <cv_bridge/cv_bridge.h>
#include "MappedPoint.h"
#include <cv_bridge/cv_bridge.h>


using namespace sensor_fusion_msg;

class FusedObject {
public:
    ObjectBoundingBox* cameraData;
    std::vector<MappedPoint> *lidarPoints;
    int r,g,b;

public:
    FusedObject();

    explicit FusedObject(ObjectBoundingBox *cameraData);

    FusedObject(ObjectBoundingBox *cameraData, std::vector<MappedPoint> *points);

    void setColor(int r, int g, int b);

    void setRandomColor();

    void addPoint(const MappedPoint& point);

    void drawObject(const cv_bridge::CvImagePtr &imagePtr);

};


#endif //PROJECT_FUSEDOBJECT_H
