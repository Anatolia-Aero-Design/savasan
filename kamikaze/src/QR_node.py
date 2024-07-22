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
        
        self.qr_pub = rospy.Publisher('/qr_code_data', String, queue_size=10)
        
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
            qr_data = obj.data.decode("utf-8")
            self.qr_pub.publish(qr_data)
        rospy.loginfo(f"QR Code Data: {qr_data}")
    
    def callback(self, frame):
        waypoints = Waypoint_node()
        waypoints.get_coordinates()
        waypoints.waypoints()
        waypoints.push_waypoints()
        self.qr_reader(frame)
    
if __name__ == '__main__':
	rospy.init_node('QR_node', anonymous=True)
	qr_node = QR_Node()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down QR node")