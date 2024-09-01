#!/usr/bin/env python

import pymap3d as pm
import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Point
from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import Quaternion
import numpy as np
import utilities as utils
from visualization_msgs.msg import Marker
import tf.transformations as tf

class PosePublisherNode:
    def __init__(self) -> None:    
        # Initialize variables
        self.uav_position = None
        self.home_pose = None
        
        # Constants
        self.TARGET_LATITUDE = 39.82006598 
        self.TARGET_LONGITUDE =  30.53404039
        self.TARGET_ALTITUDE = 0

        # Subscribers
        self.uav_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.uav_pose_callback)
        self.home_sub = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_callback)

        # Publisher
        self.vector_pub = rospy.Publisher('/uav_to_target_vector', Vector3, queue_size=10)
        self.vector_marker_pub = rospy.Publisher('/uav_to_target_marker', Marker, queue_size=10)
            
    def uav_pose_callback(self, data):
        uav_orient = utils.quaternion_to_euler(data.pose.orientation.w,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z)
        self.uav_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z,uav_orient[0],uav_orient[1],uav_orient[2]])
        self.publish_vector(data)

    def home_callback(self, data):
        self.home_pose = np.array([data.geo.latitude, data.geo.longitude, data.geo.altitude])

    def publish_vector(self, data):
        if self.uav_position is not None and self.home_pose is not None:
            try:
                x_enu, y_enu, z_enu = utils.gps_to_xyz(self.home_pose[0], self.home_pose[1], 0, self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)


                
                # Calculate the vector between the target and UAV position
                vector_body =  np.array([x_enu, y_enu, z_enu]) - np.array(self.uav_position)[:3]

                vector = Vector3()
                vector.x = vector_body[0]
                vector.y = vector_body[1]
                vector.z = vector_body[2]


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
                end_point.x = x_enu
                end_point.y = y_enu
                end_point.z = z_enu

                marker.points.append(start_point)
                marker.points.append(end_point)
                
                            # Initialize the orientation to identity quaternion
                marker.pose.orientation = Quaternion()
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

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
