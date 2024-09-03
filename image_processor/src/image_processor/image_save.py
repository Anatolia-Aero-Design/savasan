#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.msg import Yolo_xywh
import cv2
import message_filters
import time 
from std_srvs.srv import Trigger, TriggerResponse

class VideoSaveNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.recording = False

        # Subscribers using message_filters for synchronization
        self.tracking_sub = rospy.Subscriber("/camera/image_processed", Image, self.callback)
           
        competition_no = 1 # Will be set accordingly during each round of competition
        current_time = time.localtime()
        file_name = f"{competition_no}_Anatolia_Aero_Design_{current_time.tm_mday}_0{current_time.tm_mon}_{current_time.tm_year}.avi"
        
        # Initialize video writer
        self.video_writer = None
        self.video_filename = f"/home/auki/catkin_ws/src/savasan/{file_name}"
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.frame_width =  1280  # Update this with the actual width of your image
        self.frame_height = 720  # Update this with the actual height of your image
        self.fps = 30

        # Services to start and stop recording
        self.start_service = rospy.Service('image_proccessor/start_recording', Trigger, self.start_recording)
        self.stop_service = rospy.Service('image_proccessor/stop_recording', Trigger, self.stop_recording)

    def start_recording(self, req):
        try:
            self.recording = True
            rospy.loginfo("Started recording video.")
            return TriggerResponse(success=1)
        except:
            return TriggerResponse(success=0)

    def stop_recording(self, req):
        try:
            self.recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                rospy.loginfo(f"Stopped recording. Video saved as {self.video_filename}")
                self.video_writer = None  # Reset video writer for the next recording session
            return TriggerResponse(success=1)
        except e:
            rospy.logerr(e)
            return TriggerResponse(success=0)

    def callback(self, image_msg):
        if not self.recording:
            return

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
