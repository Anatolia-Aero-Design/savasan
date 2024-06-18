#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from image_processor.msg import Yolo_xywh  # Import the custom message

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=50)

        image_sub = message_filters.Subscriber('/camera/image_raw', Image)
        xywh_sub = message_filters.Subscriber('/yolov8/xywh', Yolo_xywh)

        ats = message_filters.TimeSynchronizer([image_sub, xywh_sub], queue_size=50)
        ats.registerCallback(self.callback)

    def callback(self, image_msg, xywh_msg=None):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        if xywh_msg is not None:
            x, y, w, h, trackid = xywh_msg.x, xywh_msg.y, xywh_msg.w, xywh_msg.h, xywh_msg.trackid
            cv2.rectangle(cv_image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 2)
            cv2.putText(cv_image, f'ID: {int(trackid)}', (int(x - w / 2), int(y - h / 2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(processed_image_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    rospy.init_node('image_and_xywh_processor', anonymous=True)
    ImageProcessor()
    rospy.spin()

if __name__ == '__main__':
    main()
