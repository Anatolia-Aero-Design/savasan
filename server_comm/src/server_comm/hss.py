#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from server_comm.msg import HavaSavunmaKoordinatlari
from mavros_msgs.srv import CommandInt, SetMode
from mavros_msgs.msg import CommandCode, State
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
from mavros_msgs.srv import WaypointPush
import math
import rospkg
import os
from visualization_msgs.msg import Marker, MarkerArray
import pymap3d as pm
from mavros_msgs.msg import HomePosition
import numpy as np

# Plane does not support NAV_FENCE_CIRCLE_EXCLUSION

class Air_Defense_Node:
    def __init__(self) -> None:
        
        self.home_sub = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_callback)
        self.hss_sub = rospy.Subscriber(
                "hss_locations", HavaSavunmaKoordinatlari, self.hss_callback
            )
        
        self.marker_pub = rospy.Publisher("hss_marker", MarkerArray,queue_size=10)

        # Set up service proxies
        self.waypoint_push_srv = rospy.ServiceProxy(
            '/mavros/mission/push', WaypointPush)
        
        self.push_hss_coordinates = rospy.Service(
            "push_hss_coordinates", Trigger, self.push_hss_coordinates_callback)
    def home_callback(self, data):
        self.home_pose = np.array(
            [data.geo.latitude, data.geo.longitude, data.geo.altitude]
        )
        self.HOME_LATITUDE = self.home_pose[0]
        self.HOME_LONGITUDE = self.home_pose[1]
        self.HOME_ALTITUDE = self.home_pose[2]
        return self.HOME_ALTITUDE, self.HOME_LONGITUDE, self.HOME_LATITUDE
        
        
    def hss_callback(self,data):
        self.hss_coordinates = data
        print(self.hss_coordinates)
        
    def push_hss_coordinates_callback(self,req):
        
        rospy.wait_for_service("/mavros/geofence/push")
        
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "hss"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker_array = MarkerArray()
        for data in self.hss_coordinates.hss_Koordinati:
            hssEnlem = data.hssEnlem  # Access fields from each item
            hssBoylam = data.hssBoylam
            hssYaricap = data.hssYaricap
            # Conversion from geodetic to ENU
            x, y, z = pm.geodetic2enu(hssEnlem, hssBoylam, 0, self.HOME_LATITUDE, self.HOME_LONGITUDE, self.HOME_ALTITUDE)

        # Create the marker (Cylinder)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "hss"
            marker.id = data.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Set position and orientation
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 500  # Half of the height (since height is centered)

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set scale (size) based on radius and height
            marker.scale.x = hssYaricap * 2  # Diameter in x
            marker.scale.y = hssYaricap * 2  # Diameter in y
            marker.scale.z = 1000.0  # Height in meters

            # Set color (green)
            marker.color.a = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # Append the marker to the array
            marker_array.markers.append(marker)
            self.marker_pub.publish(marker_array)
            rospy.loginfo("Published HSS markers")
            

        
        

if __name__ == '__main__':
    try:
        rospy.init_node("air_defense_node", anonymous=True)
        Air_Defense_Node()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt: {e}")