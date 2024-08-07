#!/usr/bin/env python

import pymap3d as pm
import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Point
from mavros_msgs.msg import HomePosition
import numpy as np
import utilities as utils
from visualization_msgs.msg import Marker

class Pose_Publisher_Node:
    def __init__(self) -> None:
        rospy.init_node('position_vector_publisher', anonymous=True)

        # API URLs
        self.server_url_qr_koordinati = rospy.get_param('/comm_node/api/qr_koordinati')
        
        # Subscribers
        self.uav_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.uav_pose_callback)
        self.home_sub= rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_callback)

        # Publisher
        self.vector_pub = rospy.Publisher('/uav_to_target_vector', Vector3, queue_size=10)

        self.uav_position = None
        self.home_pose = np.array([-35.3632622, 149.1652374]) 
        self.TARGET_LATITUDE = -35.360849
        self.TARGET_LONGITUDE = 149.161835
        self.TARGET_ALTITUDE = 0
            
            
    def uav_pose_callback(self, data):
        
        self.uav_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.publish_vector()

    def home_callback(self, data):
        self.home_pose = np.array([data.geo.latitude, data.geo.longitude])
        rospy.loginfo(f"Received home pose: {self.home_pose}")
        self.publish_vector()




    def publish_vector(self):
        if self.uav_position is not None and self.home_pose is not None:
            x, y, z = pm.geodetic2enu(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, 0, self.home_pose[0], self.home_pose[1], self.TARGET_ALTITUDE)
            vector = Vector3()
            vector.x = x
            vector.y = y
            vector.z = z
            self.vector_pub.publish(vector)
        else:
            rospy.logwarn("Cannot publish vector: UAV position or home pose is not available.")

            
if __name__ == '__main__':
    try:
        Pose_Publisher_Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass