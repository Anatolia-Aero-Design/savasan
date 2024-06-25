#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from std_srvs.srv import Empty, EmptyResponse
from image_processor.msg import Yolo_xywh  # Import the custom message#
import cv2
import time

class YOLOv8TrackingNode:
	def __init__(self):
		self.bridge = CvBridge()
		self.bridge2 = CvBridge()
		self.image_sub = None  # Subscriber will be created in start_tracking
		self.bbox_pub = rospy.Publisher("/yolov8/xywh", Yolo_xywh, queue_size=60)


		self.model = YOLO("/home/valvarn/catkin_ws/src/savasan/yolov8/model/best_s.pt")

		# Services to start and stop tracking
		self.start_service = rospy.Service('start_yolov8_tracking', Empty, self.start_tracking)
		self.stop_service = rospy.Service('stop_yolov8_tracking', Empty, self.stop_tracking)
		self.xyxy = Yolo_xywh() 



	def start_tracking(self, req):
		if self.image_sub is None:
			self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
			rospy.loginfo("YOLOv8 tracking started.")
		return EmptyResponse()

	def stop_tracking(self, req):
		if self.image_sub is not None:
			self.image_sub.unregister()
			self.image_sub = None
			
			if self.video_writer is not None:
				self.video_writer.release()
				
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
		results = self.model.track(frame,persist=True,stream_buffer=True,verbose=True)
		t2 = time.time()
		print(t2-t1)
		t3 = time.time()
		result = results[0].boxes.xyxy.cpu().numpy()

		
		if len(result>0):
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
		print(t4-t3)
		self.bbox_pub.publish(self.xyxy)


	def __del__(self):
		# Release the VideoWriter when the node is destroyed
		print('destroyed')
		if self.video_writer is not None:
			self.video_writer.release()


if __name__ == '__main__':
	rospy.init_node('yolov8_tracking_node', anonymous=True)
	yolov8_tracking_node = YOLOv8TrackingNode()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down YOLOv8 tracking node")
