#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.msg import Yolo_xywh  # Adjust this import based on your message definition
import cv2
import time
from std_msgs.msg import Bool, String

class ImageProcessorNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.start_time = None
        self.server_time = None
        self.image = None
        self.bbox = self.bbox_x = self.bbox_y = self.bbox_w = self.bbox_h = None
        
        # Subscribers using message_filters for synchronization
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.bbox_sub = rospy.Subscriber("/yolov8/xywh", Yolo_xywh, self.bbox_callback)  # Adjust topic and message type
        self.server_time_sub = rospy.Subscriber('/server_time', String, self.server_time_callback)
        
        # Publisher for processed images
        self.image_pub = rospy.Publisher("/camera/image_processed", Image, queue_size=60)
        self.lock_on_pub = rospy.Publisher("/lock_on_status", Bool, queue_size=10)
        self.kilit_pub = rospy.Publisher("/kilit", Bool, queue_size=10)  
    
    def image_callback(self,msg):
        self.image_msg = msg
        self.callback()
    
    def bbox_callback(self, msg):
        self.bbox = msg
        self.bbox_x = msg.x
        self.bbox_y = msg.y
        self.bbox_w = msg.w
        self.bbox_h = msg.h
        rospy.loginfo(f"Received bbox data: x={self.bbox_x}, y={self.bbox_y}, w={self.bbox_w}, h={self.bbox_h}")
    
    def server_time_callback(self, msg):
        self.server_time = msg.data
      
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
        target_coordinates = target_box_x, target_box_y, target_box_w, target_box_h
        cv2.rectangle(frame, (target_box_x, target_box_y), (target_box_w, target_box_h), (0, 255, 0), 2) 
        
        if self.bbox is not None:
        # Draw bounding box
            bbox_x, bbox_y, bbox_w, bbox_h = self.bbox.x, self.bbox.y, self.bbox.w, self.bbox.h
            if bbox_x != 0 or bbox_y != 0 or bbox_w != 0 or bbox_h != 0:
                cv2.rectangle(frame, (bbox_x, bbox_y), (bbox_w, bbox_h), (0, 0, 255), 2) # Updated to draw correctly
                bbox_coordinates = bbox_x, bbox_y, bbox_w, bbox_h
                proportions = self.calculate_lock_on_proportion(target_coordinates, bbox_coordinates)
                lock_on_status, kilit = self.lock_on_status(proportions, frame)
                self.lock_on_pub.publish(lock_on_status)
                self.kilit_pub.publish(kilit)
                
        self.overlay_server_time(frame)

        try:
            # Convert OpenCV image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            processed_image_msg.header = self.image_msg.header
            self.image_pub.publish(processed_image_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
              
    # Calculate bounding box proportion to the target area 
    def calculate_lock_on_proportion(self, target_coordinates, bbox_coordinates):
        x_t, y_t, w_t, h_t = target_coordinates
        target_area = abs(w_t - x_t) * abs(h_t - y_t)
                
        x_d, y_d, w_d, h_d = bbox_coordinates
        bbox_area = abs(w_d - x_d) * abs(h_d - y_d)
        
        if bbox_area <= target_area:
            x_o = max(x_t, x_d)
            y_o = max(y_t, y_d)
            w_o = min(w_t, w_d)
            h_o = min(h_t, h_d)

            overlap_width = max(0, w_o - x_o)
            overlap_height = max(0, h_o - y_o)
            
            horizontal_proportion = overlap_width / (w_d - x_d) if (w_d - x_d) != 0 else 0
            vertical_proportion = overlap_height / (h_d - y_d) if (h_d - y_d) != 0 else 0
        return horizontal_proportion, vertical_proportion
    
    def lock_on_status(self, proportions, frame):
        lock_on_status = False
        kilit = False
        if lock_on_status is not True and proportions[0] >= 0.93 and proportions[1] >= 0.93:
            elapsed_time = self.timer(frame)
            if elapsed_time >= 4.00:
                lock_on_status = True
                rospy.loginfo("Success")

            if elapsed_time > 0.00:
                kilit = True
        else:
            elapsed_time_text = f"Lock-on-time: 0.00"
            text_size, _ = cv2.getTextSize(elapsed_time_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2) 
            text_x = int((frame.shape[1] - text_size[0]))
            text_y = int((frame.shape[0] + text_size[1]) / 2)
            cv2.putText(frame, elapsed_time_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            self.start_time = None
            lock_on_status = False 
        return lock_on_status, kilit
    
    def timer(self, frame):
        if self.start_time is None:
            self.start_time = time.time()

        elapsed_time = time.time() - self.start_time
        elapsed_time_text = f"Lock-on-time: {elapsed_time:.2f}"
        text_size, _ = cv2.getTextSize(elapsed_time_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2) 
        text_x = int((frame.shape[1] - text_size[0]))
        text_y = int((frame.shape[0] + text_size[1]) / 2)
        cv2.putText(frame, elapsed_time_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        return elapsed_time
        
    def overlay_server_time(self, frame):
        if self.server_time:
            server_time_text = str(self.server_time)
            text_size, _ = cv2.getTextSize(server_time_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            text_x = int(frame.shape[1] - text_size[0] - 10)
            text_y = int(text_size[1] + 10)
            cv2.putText(frame, server_time_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

if __name__ == '__main__':
    rospy.init_node('image_processor_node', anonymous=True)
    image_processor_node = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Image Processor node")