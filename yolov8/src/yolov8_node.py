#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Imu
from mavros_msgs.msg import AttitudeTarget
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from std_srvs.srv import Empty, EmptyResponse
from image_processor.msg import Yolo_xywh  # Import the custom message#
import cv2
import time
import math
import numpy as np

class PID:
	def __init__(self, Kp, Ki, Kd, setpoint=0):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.setpoint = setpoint
		self.previous_error = 0
		self.integral = 0

	def update(self, current_value):
		error = self.setpoint - current_value
		self.integral += error
		derivative = error - self.previous_error
		output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
		self.previous_error = error
		return output

class YOLOv8TrackingNode:
	def __init__(self):
		self.bridge = CvBridge()
		self.bridge2 = CvBridge()
		self.image_sub = None  # Subscriber will be created in start_tracking
  
		self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu)
		self.attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
		self.bbox_pub = rospy.Publisher("/yolov8/xywh", Yolo_xywh, queue_size=60)

		self.model = YOLO("/home/valvarn/catkin_ws/src/savasan/yolov8/model/best_s.pt")

		# Services to start and stop tracking
		self.start_service = rospy.Service('start_yolov8_tracking', Empty, self.start_tracking)
		self.stop_service = rospy.Service('stop_yolov8_tracking', Empty, self.stop_tracking)
		self.xyxy = Yolo_xywh()

		# PID controllers for yaw, pitch, and roll
		self.yaw_pid = PID(Kp=1.0, Ki=0.0, Kd=0.1)
		self.pitch_pid = PID(Kp=1.0, Ki=0.0, Kd=0.1)
		self.roll_pid = PID(Kp=1.0, Ki=0.0, Kd=0.1)

		# Image window name
		self.window_name = 'YOLOv8 Tracking'

	def start_tracking(self, req):
		if self.image_sub is None:
			self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
			rospy.loginfo("YOLOv8 tracking started.")
		return EmptyResponse()

	def stop_tracking(self, req):
		if self.image_sub is not None:
			self.image_sub.unregister()
			self.image_sub = None
			rospy.loginfo("YOLOv8 tracking stopped.")
		return EmptyResponse()

	def image_callback(self, data):
		try:
			# Convert ROS Image message to OpenCV image
			frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(f"CvBridge Error: {e}")
			return

		# Run YOLOv8 tracking on the frame, persisting tracks between frames
		t1 = time.time()
		results = self.model.track(frame, persist=True, stream_buffer=True, verbose=True)
		t2 = time.time()
		print(t2 - t1)
		t3 = time.time()
		result = results[0].boxes.xyxy.cpu().numpy()

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
		print(t4 - t3)

		yaw, pitch, roll = self.calculate_yaw_pitch_roll(frame, self.xyxy, frame.shape)
		
		# Calculate control outputs using PID controllers
		self.yaw_pid.update(yaw)
		self.pitch_pid.update(pitch)
		self.roll_pid.update(roll)

		# Draw yaw, pitch, and roll values on the image
		self.draw_yaw_pitch_roll_text(frame, yaw, pitch, roll)

		# Show the image with annotations
		cv2.imshow(self.window_name, frame)
		cv2.waitKey(1)

		self.bbox_pub.publish(self.xyxy)

	def calculate_yaw_pitch_roll(self, image ,bounding_box, image_shape):
		img_height, img_width = image_shape[:2]

		# Use the attributes of the Yolo_xywh object directly
		box_x = bounding_box.x
		box_y = bounding_box.y
		box_width = bounding_box.w
		box_height = bounding_box.h

		# Calculate the center of the bounding box
		box_center_x = box_x + box_width / 2
		box_center_y = box_y + box_height / 2

		# Calculate offsets from the image center
		offset_x = box_center_x - img_width / 2
		offset_y = box_center_y - img_height / 2

		# Normalize offsets to [-1, 1] range
		norm_offset_x = offset_x / (img_width / 2)
		norm_offset_y = offset_y / (img_height / 2)

		# Calculate yaw and pitch
		yaw = norm_offset_x * 45  # Assume max yaw is ±45 degrees
		pitch = -norm_offset_y * 45  # Assume max pitch is ±45 degrees

		# Roll is not determined by bounding box in this simple example
		roll = 0
		
		target_box_x = int(1280 * 0.25)
		target_box_y = int(720 * 0.1)
		target_box_w = int(1280 * 0.75)
		target_box_h = int(720 * 0.9)

		bbox_x, bbox_y, bbox_w, bbox_h = box_x, box_y, box_width, box_height
		center_target_x = (target_box_w + target_box_x)// 2
		center_target_y = (target_box_h + target_box_y) // 2
		center_x = (bbox_w + bbox_x) // 2
		center_y = (bbox_h + bbox_y) // 2
		if bbox_x != 0 or bbox_y != 0 or bbox_w != 0 or bbox_h != 0:
			cv2.rectangle(image, (bbox_x, bbox_y), (bbox_w, bbox_h), (0, 0, 255), 2) # Updated to draw correctly
			cv2.arrowedLine(image, (center_target_x, center_target_y), (center_x, center_y), (0, 0, 255), 2)
			cv2.rectangle(image, (target_box_x, target_box_y), (target_box_w, target_box_h), (0, 255, 0), 2)
		return yaw, pitch, roll

	def send_attitude(self, yaw, pitch, roll):
		attitude = AttitudeTarget()
		attitude.type_mask = 0  # ignore rate targets

		q = self.euler_to_quaternion(roll, pitch, yaw)
		attitude.orientation.x = q[0]
		attitude.orientation.y = q[1]
		attitude.orientation.z = q[2]
		attitude.orientation.w = q[3]

		attitude.thrust = 0.5  # example thrust value
		self.attitude_pub.publish(attitude)
		rospy.loginfo(f"Published attitude: yaw={yaw}, pitch={pitch}, roll={roll}")
  
	def euler_to_quaternion(self, roll, pitch, yaw):
		qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
		qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
		qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		return [qx, qy, qz, qw]
    
	def draw_yaw_pitch_roll_text(self, image, yaw, pitch, roll):
		# Draw yaw, pitch, and roll values on the image
		cv2.putText(image, f"Yaw: {yaw:.2f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
		cv2.putText(image, f"Pitch: {pitch:.2f} deg", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		cv2.putText(image, f"Roll: {roll:.2f} deg", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		
  		

if __name__ == '__main__':
	rospy.init_node('yolov8_tracking_node', anonymous=True)
	yolov8_tracking_node = YOLOv8TrackingNode()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down YOLOv8 tracking node")
