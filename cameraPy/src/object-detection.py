#!/usr/bin/env python
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
from time import sleep

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

from darknet import *

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/detection/out", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback)
        # Give the configuration and weight files for the model and load the network using them.
        modelConfiguration = b"/home/dieter/darknet/cfg/yolov2-tiny.cfg"
        modelWeights = b"/home/dieter/darknet/data/yolov2-tiny.weights"

        self.net = load_net(modelConfiguration, modelWeights, 0)
        self.meta = load_meta(b"/home/dieter/darknet/cfg/coco.data")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Save image on temporary location for object detection
        cv2.imwrite('/tmp/tmp.png', cv_image)
        #sleep(0.2)
        r = detect(self.net, self.meta, b"/tmp/tmp.png")

        cv_image = self.drawPredicions(cv_image,r,0.6)

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
