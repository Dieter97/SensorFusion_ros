//
// Created by dieter on 25.04.19.
//

#include <fusion/fusion_objects/MappedPoint.h>

#include "fusion/fusion_objects/MappedPoint.h"

MappedPoint::MappedPoint(pcl::PointXYZ point, int width, int height, int scale, float cameraPlane, float copX, float copY, float copZ) {
    this->copX = copX; this->copY =  copY; this->copZ = copZ;
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

float MappedPoint::mapPoint() {
    float *p;
    p = perspectiveMapping(point.x, point.y, point.z, cameraPlane);
    this->pictureX = ((*(p + 1) * scale) + this->screenWidth / 2);
    this->pictureY = ((*(p + 2) * scale) + this->screenHeight / 2);
}

float * MappedPoint::perspectiveMapping(float x1, float y1, float z1, float cameraPlane) {
    float k = (cameraPlane - x1) / (x1-this->copX);
    static float result[3];
    result[0] = x1 + (k * (x1-this->copX));
    result[1] = y1 + (k * (y1-this->copY));
    result[2] = z1 + (k * (z1-this->copZ));
    return result;
}

float MappedPoint::getDistance() {
    return this->distance;
}

float MappedPoint::map(float value, float low1, float high1, float low2, float high2) {
    return (value - low1) * (high2 - low2) / (high1 - low1) + low2;
}

float MappedPoint::getPictureX() const {
    return pictureX;
}

void MappedPoint::setPictureX(float pictureX) {
    MappedPoint::pictureX = pictureX;
}

float MappedPoint::getPictureY() const {
    return pictureY;
}

void MappedPoint::setPictureY(float pictureY) {
    MappedPoint::pictureY = pictureY;
}

pcl::PointXYZ MappedPoint::getPCLPoint() {
    return this->point;
}

MappedPoint::MappedPoint(const MappedPoint &point) {
    this->point = point.point;
    this->distance = point.distance;
    this->screenHeight = point.screenHeight;
    this->screenWidth = point.screenWidth;
    this->pictureX = point.pictureX;
    this->pictureY = point.pictureY;
    this->cameraPlane = point.cameraPlane;
}

float MappedPoint::getCameraPlane() const {
    return cameraPlane;
}

int MappedPoint::getScreenWidth() const {
    return screenWidth;
}

int MappedPoint::getScreenHeight() const {
    return screenHeight;
}

int MappedPoint::getScale() const {
    return scale;
}

float MappedPoint::getCopX() const {
    return copX;
}

float MappedPoint::getCopY() const {
    return copY;
}

float MappedPoint::getCopZ() const {
    return copZ;
}
