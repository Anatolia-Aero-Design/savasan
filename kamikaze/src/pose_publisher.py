#!/usr/bin/env python

import pymap3d as pm
import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Point
from mavros_msgs.msg import HomePosition
import numpy as np
import utilities as utils
from visualization_msgs.msg import Marker

class PosePublisherNode:
    def __init__(self) -> None:    
        # Initialize variables
        self.uav_position = None
        self.home_pose = None
        
        # Constants
        self.TARGET_LATITUDE = -35.360849
        self.TARGET_LONGITUDE = 149.161835
        self.TARGET_ALTITUDE = 0

        # Subscribers
        self.uav_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.uav_pose_callback)
        self.home_sub = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_callback)

        # Publisher
        self.vector_pub = rospy.Publisher('/uav_to_target_vector', Vector3, queue_size=10)
        self.vector_marker_pub = rospy.Publisher('/uav_to_target_marker', Marker, queue_size=10)
            
    def uav_pose_callback(self, data):
        self.uav_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.publish_vector()

    def home_callback(self, data):
        self.home_pose = np.array([data.geo.latitude, data.geo.longitude, data.geo.altitude])
        self.publish_vector()

    def publish_vector(self):
        if self.uav_position is not None and self.home_pose is not None:
            try:
                x, y, z = pm.geodetic2enu(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE, self.home_pose[0], self.home_pose[1], self.home_pose[2])
                vector = Vector3()
                vector.x = x
                vector.y = y
                vector.z = z

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "vector"
                marker.id = 0
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                start_point = Point()
                start_point.x = self.uav_position[0]
                start_point.y = self.uav_position[1]
                start_point.z = self.uav_position[2]

                end_point = Point()
                end_point.x = x
                end_point.y = y
                end_point.z = self.TARGET_ALTITUDE

                marker.points.append(start_point)
                marker.points.append(end_point)

                marker.scale.x = 0.5  # Shaft diameter
                marker.scale.y = 0.3   # Head diameter
                marker.scale.z = 0.3   # Head length

                marker.color.a = 1.0  # Alpha
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0  # Green
                marker.color.b = 0.0  # Blue

                self.vector_marker_pub.publish(marker)
                self.vector_pub.publish(vector)
                
            except Exception as e:
                rospy.logerr(f"Failed to publish vector: {e}")
        else:
            rospy.logwarn("UAV position or home pose not available yet.")
            
if __name__ == '__main__':
    try:
        rospy.init_node('pose_publisher_node', anonymous=True)
        PosePublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
