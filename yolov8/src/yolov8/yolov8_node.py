#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Imu
from mavros_msgs.msg import AttitudeTarget
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from std_srvs.srv import Trigger,TriggerResponse
from image_processor.msg import Yolo_xywh  # Import the custom message#
import cv2
import time
import math
import numpy as np
import rospkg
import os


class YOLOv8TrackingNode:
    def __init__(self):
        self.prev_time = rospy.get_time()
        self.bridge = CvBridge()
        self.image_sub = None  # Subscriber will be created in start_tracking

        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu)
        self.attitude_pub = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.bbox_pub = rospy.Publisher(
            "/yolov8/xywh", Yolo_xywh, queue_size=60)
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('yolov8')
        file_path = os.path.join(package_path, 'model/best_s.pt')

        self.model = YOLO(file_path)

        # Services to start and stop tracking
        self.start_service = rospy.Service(
            'start_yolov8_tracking', Trigger, self.start_tracking)
        self.stop_service = rospy.Service(
            'stop_yolov8_tracking', Trigger, self.stop_tracking)
        self.xyxy = Yolo_xywh()

    def start_tracking(self, req):
        try:
            if self.image_sub is None:
                self.image_sub = rospy.Subscriber(
                    "/camera/image_raw", Image, self.image_callback)
                rospy.loginfo("YOLOv8 tracking started.")
            return TriggerResponse(success=1)
        except:
            return TriggerResponse(success=0)


    def stop_tracking(self, req):
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None
                time.sleep(0.5)
                self.xyxy.header.stamp = rospy.Time.now()
                self.xyxy.x = 0
                self.xyxy.y = 0
                self.xyxy.w = 0
                self.xyxy.h = 0
                self.bbox_pub.publish(self.xyxy)
                rospy.loginfo("YOLOv8 tracking stopped.")
            return TriggerResponse(success=1)
        except:
            return TriggerResponse(success=0)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        t1 = time.time()
        results = self.model.track(
            frame, persist=True, stream_buffer=True, verbose=False)
        t2 = time.time()
        t3 = time.time()
        result = results[0].boxes.xyxy.cpu().numpy()
        try:
            track_ids = results[0].boxes.id.int().cpu().tolist()
        except AttributeError:
            pass
        frame = results[0].plot()

        if len(result > 0):
            self.xyxy.header.stamp = data.header.stamp
            self.xyxy.x = int(result[0][0])
            self.xyxy.y = int(result[0][1])
            self.xyxy.w = int(result[0][2])
            self.xyxy.h = int(result[0][3])
        else:
            self.xyxy.header.stamp = data.header.stamp
            self.xyxy.x = 0
            self.xyxy.y = 0
            self.xyxy.w = 0
            self.xyxy.h = 0
        t4 = time.time()

        self.bbox_pub.publish(self.xyxy)


if __name__ == '__main__':
    rospy.init_node('yolov8_tracking_node', anonymous=True)
    yolov8_tracking_node = YOLOv8TrackingNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLOv8 tracking node")
