//
// Created by dieter on 27.04.19.
//

#include "fusion/fusion_objects/DarknetObject.h"


DarknetObject::DarknetObject(char *cfg, char *weights, int clear, char *file){

    void* handle = dlopen("/home/dieter/darknet/libdarknet.so", RTLD_GLOBAL | RTLD_LAZY);
    dlerror();

    // Find all functions
    *(void**) (&load_network) = dlsym(handle,"load_network");
    *(void**) (&get_metadata) = dlsym(handle,"get_metadata");
    *(void**) (&load_image) = dlsym(handle,"load_image");
    *(void**) (&network_predict_image) = dlsym(handle,"network_predict_image");
    *(void**) (&get_network_boxes) = dlsym(handle,"get_network_boxes");
    *(void**) (&do_nms_obj) = dlsym(handle,"do_nms_obj");
    *(void**) (&free_detections) = dlsym(handle,"free_detections");
    *(void**) (&free_image) = dlsym(handle,"free_image");

    this->net = load_network(cfg,weights,clear);
    this->meta = get_metadata(file);
}

std::vector<sensor_fusion_msg::ObjectBoundingBox> DarknetObject::detect_objects(char * im, float threshold, float hier_tresh, float nms){
    image _image = load_image(im,0,0,3);
    int num = 0;
    int * pnum = &num;
    network_predict_image(net,_image);
    detection * dets = get_network_boxes(net,_image.w,_image.h,threshold,hier_tresh, nullptr,0,pnum);
    num = pnum[0];
    do_nms_obj(dets,num,this->meta.classes,nms);

    std::vector<sensor_fusion_msg::ObjectBoundingBox> result;
    for(int j=0; j < num; j++ ){
        for(int i=0; i < this->meta.classes; i++) {
            if(dets[j].prob[i] > 0){
                sensor_fusion_msg::ObjectBoundingBox box;
                box.x = dets[j].bbox.x;
                box.y = dets[j].bbox.y;
                box.w = dets[j].bbox.w;
                box.h = dets[j].bbox.h;
                box.probability = dets[j].prob[i];
                box.Class = this->meta.names[i];
                result.emplace_back(box);
            }
        }
        detection d = dets[j];
    }

    free_image(_image);
    free_detections(dets,num);
    return result;
}
