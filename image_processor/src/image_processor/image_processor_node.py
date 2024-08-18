#!/usr/bin/env python

from calendar import LocaleHTMLCalendar
from tkinter import NO
from tracemalloc import start
from pygame import ver
from sympy import N
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.msg import Yolo_xywh  # Adjust this import based on your message definition
import cv2
import message_filters
import time
from std_msgs.msg import Int32, Bool, String
import json
import requests
import logging



class ImageProcessorNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.start_time = None
        self.server_time = None
        
        # Subscribers using message_filters for synchronization
        image_sub = message_filters.Subscriber("/camera/image_raw", Image)
        bbox_sub = message_filters.Subscriber("/yolov8/xywh", Yolo_xywh)  # Adjust topic and message type
        self.server_time_sub = rospy.Subscriber('/server_time', String, self.server_time_callback)

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, bbox_sub], 60, 0.001)
        self.ts.registerCallback(self.callback)

        # Publisher for processed images
        self.image_pub = rospy.Publisher("/camera/image_processed", Image, queue_size=60)
        self.lock_on_pub = rospy.Publisher("/lock_on_status", Bool, queue_size=10)
        self.kilit_pub = rospy.Publisher("/kilit", Bool, queue_size=10)
        
        self.server_time = None
       


    def callback(self, image_msg, bbox_msg):


        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        
        self.server_time_printer(frame)

        # Draw bounding box
        bbox_x, bbox_y, bbox_w, bbox_h = bbox_msg.x, bbox_msg.y, bbox_msg.w, bbox_msg.h
        if bbox_x != 0 or bbox_y != 0 or bbox_w != 0 or bbox_h != 0:
            cv2.rectangle(frame, (bbox_x, bbox_y), (bbox_w, bbox_h), (0, 0, 255), 2) # thickness max value must be 3
                                                                                     # color must be red (0, 0, 255)                                                                        
        # Draw target area
        target_box_x = int(1280 * 0.25)
        target_box_y = int(720 * 0.1)
        target_box_w = int(1280 * 0.75)
        target_box_h = int(720 * 0.9)
        cv2.rectangle(frame, (target_box_x, target_box_y), (target_box_w, target_box_h), (0, 255, 0), 2) # thickness max value must be 3
        
        bbox_coordinates = bbox_x, bbox_y, bbox_w, bbox_h
        target_coordinates = target_box_x, target_box_y, target_box_w, target_box_h
        proportions = self.calculate_lock_on_proportion(target_coordinates, bbox_coordinates)
        lock_on_status, kilit = self.lock_on_status(proportions, frame)

        try:
            # Convert OpenCV image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            processed_image_msg.header = image_msg.header
            self.image_pub.publish(processed_image_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

        self.lock_on_pub.publish(lock_on_status)
        self.kilit_pub.publish(kilit)
              
    # Calculate bounding box proportion to the target area 
    def calculate_lock_on_proportion(self, target_coordinates, bbox_coordinates):
        # Coordinates for the target area 
        x_t, y_t, w_t, h_t = target_coordinates
        target_area = abs(w_t - x_t) * abs(h_t - y_t)
                
        # Coordinates for the bounding box
        x_d, y_d, w_d, h_d = bbox_coordinates
        bbox_area = abs(w_d - x_d) * abs(h_d - y_d)
        
        if bbox_area <= target_area:
            # Calculate overlap box coordinates
            x_o = max(x_t, x_d)
            y_o = max(y_t, y_d)
            w_o = min(w_t, w_d)
            h_o = min(h_t, h_d)

            # Calculate the width and height of the overlap box
            overlap_width = max(0, w_o - x_o)
            overlap_height = max(0, h_o - y_o)
            
            # Calculate the horizontal and vertical proportion
            horizontal_proportion = overlap_width / (w_d - x_d) if (w_d - x_d) != 0 else 0
            vertical_proportion = overlap_height / (h_d - y_d) if (h_d - y_d) != 0 else 0
        return horizontal_proportion, vertical_proportion
    
    def lock_on_status(self, proportions, image_msg):
        lock_on_status = False
        kilit = False
        if lock_on_status is not True and proportions[0] >= 0.93 and proportions[1] >= 0.93: # 0.05 is error threshold for proportions
            elapsed_time = self.timer(image_msg) # start timer when contact is made
            if elapsed_time >= 4.00:
                lock_on_status = True
                print("Success") # TODO success message will be sent to comm and package will be sent to server from comm

            if elapsed_time > 0.00:
                kilit = True

        else:
            elapsed_time_text = f"Lock-on-time: 0.00"
            text_size, _ = cv2.getTextSize(elapsed_time_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2) 
            text_x = int((image_msg.shape[1] - text_size[0]))
            text_y = int((image_msg.shape[0] + text_size[1]) / 2)
            cv2.putText(image_msg, elapsed_time_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            self.start_time = None
            lock_on_status = False 
        return lock_on_status, kilit
    
    def timer(self, image_msg):
        if self.start_time is None:
            self.start_time = time.time()  # Initialize the start time only once

        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time
        elapsed_time_text = f"Lock-on-time: {elapsed_time:.2f}"
        text_size, _ = cv2.getTextSize(elapsed_time_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2) 
        text_x = int((image_msg.shape[1] - text_size[0]))
        text_y = int((image_msg.shape[0] + text_size[1]) / 2)
        cv2.putText(image_msg, elapsed_time_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        return elapsed_time
    
    def server_time_callback(self, msg):
        self.server_time = msg.data
        rospy.loginfo(f"Received server time:")

    def server_time_printer(self, image_msg):
      if self.server_time:
        rospy.loginfo(f"Printing server time")
        server_time_text = str(self.server_time)
        text_size, _ = cv2.getTextSize(server_time_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        # Calculate the position for the top-right corner
        text_x = int(image_msg.shape[1] - text_size[0] - 10)  # 10 pixels padding from right edge
        text_y = int(text_size[1] + 10)  # 10 pixels padding from top edge

        # Put the text on the image
        cv2.putText(image_msg, server_time_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        

if __name__ == '__main__':


    rospy.init_node('image_processor_node', anonymous=True)
    image_processor_node = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down BBox Drawer node")
