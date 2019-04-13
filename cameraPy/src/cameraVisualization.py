#!/usr/bin/env python
from __future__ import print_function

import os
import sys
# import roslib
# roslib.load_manifest('my_package')
import time
from math import floor

import cv2
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_fusion_msg.msg import CameraObjects
from sensor_fusion_msg.msg import ObjectBoundingBox


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/detection/out/image", Image)

        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, queue_size = 10)
        self.prediction_seb = message_filters.Subscriber("/camera/detection/out/", CameraObjects, queue_size=1)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.prediction_seb], 10, slop=0.000000001)
        self.ts.registerCallback(self.callback)

    def callback(self, image, predictions):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            except CvBridgeError as e:
                print(e)

            cv_image = self.drawPredicions(cv_image, predictions)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

    def drawPredicions(self,image, predictions):
        for prediction in predictions.bounding_boxes:
            x = int(prediction.xmin)
            y = int(prediction.xmax)
            w = int(prediction.ymin)
            h = int(prediction.ymax)
            cv2.rectangle(image,(x-w,y-h),(x+w,y+h),(0,255,0),2)
            font = cv2.FONT_HERSHEY_PLAIN
            cv2.putText(image,prediction.Class,(x-w,y-h-5),font,2,(255,255,0))
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
