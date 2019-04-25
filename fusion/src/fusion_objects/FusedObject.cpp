//
// Created by dieter on 25.04.19.
//

#include <fusion/fusion_objects/FusedObject.h>

#include "fusion/fusion_objects/FusedObject.h"

FusedObject::FusedObject(){
    this->cameraData = nullptr;
    this->lidarPoints = new std::vector<MappedPoint>;
}

FusedObject::FusedObject(ObjectBoundingBox *cameraData) {
    this->cameraData = cameraData;
    this->lidarPoints = new std::vector<MappedPoint>;
    this->setRandomColor();
}

FusedObject::FusedObject(ObjectBoundingBox *cameraData, std::vector<MappedPoint> *points) {
    this->cameraData = cameraData;
    this->lidarPoints = points;
}

void FusedObject::setColor(int r, int g, int b) {
    this->r = r;
    this->g = g;
    this->b = b;
}

void FusedObject::setRandomColor(){
    this->r = std::rand() % 255;;
    this->g = std::rand() % 255;;
    this->b = std::rand() % 255;;
}

void FusedObject::addPoint(const MappedPoint& point) {
    this->lidarPoints->emplace_back(point);
}

void FusedObject::drawObject(const cv_bridge::CvImagePtr &imagePtr) {
    if(this->cameraData != nullptr) {
        //Draw the bounding box first
        cv::Point pt1((int)(cameraData->x-cameraData->w/2), (int)(cameraData->y-cameraData->h/2));
        cv::Point pt2((int)(cameraData->x+cameraData->w/2), (int)(cameraData->y+cameraData->h/2));
        cv::rectangle(imagePtr->image, pt1, pt2, cv::Scalar(b,g,r));
    }

    //Draw the lidar points
    for (auto it = lidarPoints->begin(); it != lidarPoints->end(); it++) {
        int thickness = 6 - (int) it->map(it->getDistance(), 0, 100, 1, 5);
        cv::circle(imagePtr->image, cv::Point((int) it->getPictureX(), (int) it->getPictureY()),
                   thickness,
                   cv::Scalar(b,g,r), cv::FILLED, cv::LINE_8);
    }
}