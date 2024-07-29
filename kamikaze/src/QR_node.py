#!/usr/bin/env python

from typing import Any
import rospy
from sensor_msgs.msg import Image, Imu
from mavros_msgs.msg import AttitudeTarget
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from waypoint import Waypoint_node
from pyzbar.pyzbar import decode


class QR_Node:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.image_sub = None # Initialize image subscriber as None
        self.qr_data = None
        
        self.qr_pub = rospy.Publisher('/qr_code_data', String, queue_size=10)
        self.image_pub = rospy.Publisher('camera/kamikaze_image', Image, queue_size=60)
        
        # Services to start and stop the mission
        self.start_service = rospy.Service('start_kamikaze', Empty, self.start_mission) 
        self.stop_service = rospy.Service('stop_kamikaze', Empty, self.stop_mission)
    
    def start_mission(self, req):
        if self.image_sub is None:
            self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
            rospy.loginfo("Kamikaze mission started.")
        return EmptyResponse()
    
    def stop_mission(self, req):
        pass
    
    def qr_reader(self, frame):
        decoded_objects = decode(frame)
        for obj in decoded_objects:
            self.qr_data = obj.data.decode("utf-8")
            self.qr_pub.publish(self.qr_data)
        rospy.loginfo(f"QR Code Data: {self.qr_data}")
    
    def callback(self, image_msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        
        waypoints = Waypoint_node()
        waypoints.waypoints()
        self.qr_reader(frame)
        
        try:
            # Convert OpenCV image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            processed_image_msg.header = image_msg.header
            self.image_pub.publish(processed_image_msg)
        
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
    
if __name__ == '__main__':
	rospy.init_node('QR_node', anonymous=True)
	qr_node = QR_Node()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down QR node")