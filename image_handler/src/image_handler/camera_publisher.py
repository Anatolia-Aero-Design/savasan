#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospkg
import os


def camera_publisher():
    rospy.init_node("camera_publisher", anonymous=True)
    pub = rospy.Publisher("camera/image_raw", Image, queue_size=10)

    bridge = CvBridge()
    while 1:
        gst_pipeline = "udpsrc port=5600 ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 !\
                        rtph264depay ! avdec_h264 ! videoconvert ! appsink"  # used for connecting to gazebo sim camera. This path is given to VideoCapture()

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("image_handler")
        file_path = os.path.join(package_path, "video/FoggyFPVHeaven.mp4")
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            rospy.logerr("Cannot open camera")
            return
        # Get the camera's frame rate
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps == 0:  # if the FPS value is not valid, set a default value
            fps = 30
            rospy.logwarn("Unable to fetch FPS from camera, setting default FPS to 30")

        rospy.loginfo(f"Camera FPS: {fps}")
        rate = rospy.Rate(fps)  # Set the ROS rate to match the camera FPS

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            height, width = frame.shape[:2]
            rospy.set_param("/camera_publisher/screen_width", width)
            rospy.set_param("/camera_publisher/screen_height", height)
            
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
        else:
            break
        cap.release()


if __name__ == "__main__":
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
