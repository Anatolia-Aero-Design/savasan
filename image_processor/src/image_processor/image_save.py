#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.msg import Yolo_xywh
import cv2
import message_filters

class VideoSaveNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribers using message_filters for synchronization
        image_sub = message_filters.Subscriber("/camera/image_raw", Image)
        bbox_sub = message_filters.Subscriber("/yolov8/xywh", Yolo_xywh)

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, bbox_sub], 100, 0.025)
        self.ts.registerCallback(self.callback)

        # Publisher for processed images
        self.image_pub = rospy.Publisher("/camera/image_processed", Image, queue_size=10)

        # Initialize video writer
        self.video_writer = None
        self.video_filename = "output_video.mp4"
        self.fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        self.frame_width = 640  # Update this with the actual width of your image
        self.frame_height = 480  # Update this with the actual height of your image
        self.fps = 30

    def callback(self, image_msg, bbox_msg):
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

        # Draw bounding box
        x, y, w, h = bbox_msg.x, bbox_msg.y, bbox_msg.w, bbox_msg.h
        if x != 0 or y != 0 or w != 0 or h != 0:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Write the frame to the video file
        self.video_writer.write(frame)

        # Convert the processed frame back to ROS Image message and publish
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(processed_image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def shutdown(self):
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo(f"Video saved as {self.video_filename}")


if __name__ == '__main__':
    rospy.init_node('VideoSaveNode', anonymous=True)
    bbox_drawer_node = VideoSaveNode()
    rospy.on_shutdown(bbox_drawer_node.shutdown)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down VideoSaveNode node")
