#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError

# Adjust this import based on your message definition
from image_processor.msg import Yolo_xywh
import cv2
from std_msgs.msg import String


class ImageProcessorNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.server_time = None
        self.image = None
        self.bbox = self.bbox_x = self.bbox_y = self.bbox_w = self.bbox_h = None
        self.elapsed_time = "00:00.00"

        # Subscribers using message_filters for synchronization
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.image_callback
        )
        # Adjust topic and message type
        self.bbox_sub = rospy.Subscriber(
            "/yolov8/xywh", Yolo_xywh, self.bbox_callback)

        # Publisher for processed images
        self.image_pub = rospy.Publisher(
            "/camera/image_processed", Image, queue_size=60
        )

    def image_callback(self, msg):
        self.image_msg = msg
        self.callback()

    def bbox_callback(self, msg):
        self.bbox = msg
        self.bbox_x = msg.x
        self.bbox_y = msg.y
        self.bbox_w = msg.w
        self.bbox_h = msg.h
        # rospy.loginfo(f"Received bbox data: x={self.bbox_x}, y={self.bbox_y}, w={self.bbox_w}, h={self.bbox_h}")

    def callback(self):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Draw target area
        target_box_x = int(1280 * 0.25)
        target_box_y = int(720 * 0.1)
        target_box_w = int(1280 * 0.75)
        target_box_h = int(720 * 0.9)
        cv2.rectangle(
            frame,
            (target_box_x, target_box_y),
            (target_box_w, target_box_h),
            (0, 255, 0),
            2,
        )
        self.overlay_server_time(frame)
        self.elapsed_time = rospy.get_param("elapsed_time", "00:00.00")
        self.overlay_timer_info(frame)

        if self.bbox is not None:
            # Draw bounding box
            bbox_x, bbox_y, bbox_w, bbox_h = (
                self.bbox.x,
                self.bbox.y,
                self.bbox.w,
                self.bbox.h,
            )
            if bbox_x != 0 or bbox_y != 0 or bbox_w != 0 or bbox_h != 0:
                # Updated to draw correctly
                cv2.rectangle(frame, (bbox_x, bbox_y),
                              (bbox_w, bbox_h), (0, 0, 255), 2)

        try:
            # Convert OpenCV image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            processed_image_msg.header = self.image_msg.header
            self.image_pub.publish(processed_image_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def overlay_server_time(self, frame):
        now = datetime.now()
        formatted_time = f"{now.hour}:{now.minute}:{now.second}:{now.microsecond}"
        text_size, _ = cv2.getTextSize(
            formatted_time, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
        )
        text_x = int(frame.shape[1] - text_size[0] - 10)
        text_y = int(text_size[1] + 10)
        cv2.putText(
            frame,
            formatted_time,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            2,
        )
        
    def overlay_timer_info(self, frame):
        # Define the font and size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5  # Increase this value for a larger font
        font_thickness = 4
        font_color = (0, 0, 0)  # Text color (green)
        
        # Get the size of the text
        text_size, _ = cv2.getTextSize(self.elapsed_time, font, font_scale, font_thickness)
        
        # Define the target box coordinates
        target_box_x = int(1280 * 0.25)
        target_box_y = int(720 * 0.1)
        target_box_w = int(1280 * 0.75)
        target_box_h = int(720 * 0.9)
        
        # Position the text right outside the right side of the target box
        text_x = target_box_w + 10  # 10 pixels to the right of the box
        text_y = target_box_y + (target_box_h - target_box_y + text_size[1]) // 2
        
        # Overlay the text
        cv2.putText(
            frame,
            self.elapsed_time,
            (text_x, text_y),
            font,
            font_scale,
            font_color,
            font_thickness,
        )




if __name__ == "__main__":
    rospy.init_node("image_processor_node", anonymous=True)
    image_processor_node = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Image Processor node")
