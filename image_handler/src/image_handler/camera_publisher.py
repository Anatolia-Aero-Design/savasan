#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def camera_publisher():
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)

    bridge = CvBridge()
    
    # GStreamer pipeline for capturing the video stream
    gst_pipeline = ("udpsrc port=5600 ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 !\
                        rtph264depay ! avdec_h264 ! videoconvert ! appsink")
        
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    

    if not cap.isOpened():
        rospy.logerr("Cannot open GStreamer pipeline")
        return

    rate = rospy.Rate(120)  # Set the ROS rate to match the camera FPS
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Cannot read frame")
            break

        try:
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = rospy.Time.now()
            pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass