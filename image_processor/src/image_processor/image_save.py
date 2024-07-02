#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.msg import Yolo_xywh
import cv2
import message_filters
import time 

class VideoSaveNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribers using message_filters for synchronization
        image_sub = rospy.Subscriber("/camera/image_processed", Image, self.callback)
        competition_no = 1 # Will be set accordingly during each round of competition
        current_time = time.localtime()
        file_name = f"{competition_no}_Anatolia_Aero_Design_{current_time.tm_mday}_0{current_time.tm_mon}_{current_time.tm_year}.avi"
        # Initialize video writer
        self.video_writer = None
        self.video_filename = f"/home/valvarn/catkin_ws/src/savasan/{file_name}"
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.frame_width =  1280  # Update this with the actual width of your image
        self.frame_height = 720  # Update this with the actual height of your image
        self.fps = 30

    def callback(self, image_msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Initialize video writer if not already initialized
        if self.video_writer is None:
            self.video_writer = cv2.VideoWriter(self.video_filename, self.fourcc, self.fps,
                                                (self.frame_width, self.frame_height))

        # Write the frame to the video file
        self.video_writer.write(frame)

    def shutdown(self):
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo(f"Video saved as {self.video_filename}")

if __name__ == '__main__':
    rospy.init_node('VideoSaveNode', anonymous=True)
    save_node = VideoSaveNode()
    rospy.on_shutdown(save_node.shutdown)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down VideoSaveNode node")