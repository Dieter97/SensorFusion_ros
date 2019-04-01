#!/usr/bin/env python
from __future__ import print_function

import os
import sys
# import roslib
# roslib.load_manifest('my_package')
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_fusion_msg.msg import CameraObjects
from sensor_fusion_msg.msg import ObjectBoundingBox
sys.path.append(os.path.join(os.getcwd(),'python/'))

import darknet as dn

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/detection/out", CameraObjects)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback, buff_size=100, queue_size=30)
        # Give the configuration and weight files for the model and load the network using them.
        modelConfiguration = b"/home/dieter/darknet/cfg/yolov3.cfg"
        modelWeights = b"/home/dieter/darknet/data/yolov3.weights"

        dn.set_gpu(0)

        self.net = dn.load_net(modelConfiguration, modelWeights, 0)
        self.meta = dn.load_meta(b"/home/dieter/darknet/cfg/coco.data")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Save image on temporary location for object detection
        cv2.imwrite('/tmp/tmp.png', cv_image)
        start = time.time()
        r = dn.detect(self.net, self.meta, b"/tmp/tmp.png")
        end = time.time()
        self.generateMsg(r, 0.6)

        print("YOLO prediction took %f seconds" % (end - start))

    def generateMsg(self, predictions, threshold):
        objects = []
        for prediction in predictions:
            if prediction[1] > threshold:
                object = ObjectBoundingBox()
                object.Class = prediction[0]
                object.probability = prediction[1]
                object.xmin = int(prediction[2][0])
                object.xmax = int(prediction[2][1])
                object.ymin = int(prediction[2][2])
                object.ymax = int(prediction[2][3])
                objects.append(object)
        objectMsg = CameraObjects()
        objectMsg.header = Header()
        objectMsg.header.stamp = rospy.Time.now()
        objectMsg.bounding_boxes = objects
        self.image_pub.publish(objectMsg)

    def drawPredicions(self,image,predictions,threshold):
        for prediction in predictions:
            if prediction[1] > threshold:
                x = int(prediction[2][0])
                y = int(prediction[2][1])
                w = int(prediction[2][2])
                h = int(prediction[2][3])
                cv2.rectangle(image,(x-w,y-h),(x+w,y+h),(0,255,0),2)
                font = cv2.FONT_HERSHEY_PLAIN
                cv2.putText(image,prediction[0],(x-w,y-h-5),font,2,(255,255,0))
        return image


    def array_to_image(self,arr):
        arr = arr.transpose(2,0,1)
        c = arr.shape[0]
        h = arr.shape[1]
        w = arr.shape[2]
        arr = (arr/255.0).flatten()
        data = dn.c_array(dn.c_float, arr)
        im = dn.IMAGE(w,h,c,data)
        return im

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
