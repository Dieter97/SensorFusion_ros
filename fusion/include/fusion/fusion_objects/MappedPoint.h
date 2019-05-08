//
// Created by dieter on 25.04.19.
//

#ifndef PROJECT_MAPPEDPOINT_H
#define PROJECT_MAPPEDPOINT_H


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

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
    float copX,copY,copZ;

public:
    MappedPoint(pcl::PointXYZ point, int width, int height, int scale, float cameraPlane);

    //Copy constructor
    MappedPoint(const MappedPoint &point);

    /*~MappedPoint() {
        delete(&this->point);
    }*/

    float mapPoint();

    float *perspectiveMapping(float x1, float y1, float z1, float cameraPlane);

    float getDistance();

    float map(float value, float low1, float high1, float low2, float high2);

    float getPictureX() const;

    void setPictureX(float pictureX);

    float getPictureY() const;

    void setPictureY(float pictureY);

    pcl::PointXYZ getPCLPoint();

};


#endif //PROJECT_MAPPEDPOINT_H
