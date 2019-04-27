//
// Created by dieter on 27.04.19.
//

#ifndef PROJECT_DARKNET_H
#define PROJECT_DARKNET_H

#include <darknet.h>
#include <dlfcn.h>
#include <sensor_fusion_msg/ObjectBoundingBox.h>

class DarknetObject {
private:
    network* net;
    metadata meta;
    network* (*load_network)(char*, char*, int);
    metadata (*get_metadata)(char*);
    image (*load_image)(char*,int,int,int);
    float* (*network_predict_image)(network*,image);
    detection* (*get_network_boxes)(network*,int,int,float,float,int*,int,int*);
    void (*do_nms_obj)(detection*,int,int,float);
    void (*free_detections)(detection*,int);
    void (*free_image)(image);

public:
    DarknetObject(char *cfg, char *weights, int clear, char *file);

    std::vector<sensor_fusion_msg::ObjectBoundingBox> detect_objects(char * im, float threshold, float hier_tresh, float nms);
};


#endif //PROJECT_DARKNET_H
