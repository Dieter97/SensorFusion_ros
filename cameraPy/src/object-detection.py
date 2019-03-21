#!/usr/bin/env python
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
import time

import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os.path

import sys, os
sys.path.append(os.path.join(os.getcwd(),'python/'))

import darknet as dn

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/detection/out", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback,queue_size=1)
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
        start = time.time()

        # Save image on temporary location for object detection
        cv2.imwrite('/tmp/tmp.png', cv_image)
        #im = self.array_to_image(cv_image)
        #dn.rgbgr_image(im)

        r = dn.detect(self.net, self.meta, b"/tmp/tmp.png")

        #r = dn.detect2(self.net, self.meta, im)
        cv_image = self.drawPredicions(cv_image,r,0.6)

        end = time.time()
        print("YOLO prediction took %f seconds" % (end - start))

        #cv2.imshow("Image window",  self.cv_image)
        #cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

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
