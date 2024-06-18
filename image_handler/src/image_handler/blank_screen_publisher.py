#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

def blank_screen_publisher():
    rospy.init_node('blank_screen_publisher', anonymous=True)
    pub = rospy.Publisher('camera/blank_image', Image, queue_size=10)
    rate = rospy.Rate(30)  # 10hz
    bridge = CvBridge()

    height, width = 1920, 1080  # Example dimensions
    blank_image = np.zeros((height, width, 3), np.uint8)

    while not rospy.is_shutdown():
        try:
            ros_image = bridge.cv2_to_imgmsg(blank_image, "bgr8")
            pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        blank_screen_publisher()
    except rospy.ROSInterruptException:
        pass
