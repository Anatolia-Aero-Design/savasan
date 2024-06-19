#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.msg import Yolo_xywh
import cv2
import message_filters

class BBoxDrawerNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribers using message_filters for synchronization
        image_sub = message_filters.Subscriber("/camera/image_raw", Image)
        bbox_sub = message_filters.Subscriber("/yolov8/xywh", Yolo_xywh)

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, bbox_sub], 60, 0.001)
        self.ts.registerCallback(self.callback)

        # Publisher for processed images
        self.image_pub = rospy.Publisher("/camera/image_processed", Image, queue_size=60)
        


    def callback(self, image_msg, bbox_msg):

        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Draw bounding box
        x, y, w, h = bbox_msg.x, bbox_msg.y, bbox_msg.w, bbox_msg.h
        if x != 0 or y != 0 or w != 0 or h != 0:
            cv2.rectangle(frame, (x, y), (w, h), (0, 255, 0), 2)


        try:
            # Convert OpenCV image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            processed_image_msg.header = image_msg.header
            self.image_pub.publish(processed_image_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

if __name__ == '__main__':
    rospy.init_node('bbox_drawer_node', anonymous=True)
    bbox_drawer_node = BBoxDrawerNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down BBox Drawer node")
