#!/usr/bin/env python

from urllib import response
import rospy
from sensor_msgs.msg import NavSatFix
from server_comm.msg import HavaSavunmaKoordinatlari
from mavros_msgs.srv import CommandInt, SetMode
from mavros_msgs.msg import CommandCode, State
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
from mavros_msgs.srv import WaypointPush, WaypointPushRequest
from mavros_msgs.msg import Waypoint

import math
import rospkg
import os
from visualization_msgs.msg import Marker, MarkerArray
import pymap3d as pm
from mavros_msgs.msg import HomePosition
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import pymap3d as pm
from mavros_msgs.msg import HomePosition
import numpy as np

# Plane does not support NAV_FENCE_CIRCLE_EXCLUSION


class Air_Defense_Node:
    def __init__(self) -> None:

        self.hss_sub = rospy.Subscriber(
            "hss_locations", HavaSavunmaKoordinatlari, self.hss_callback
        )

        self.marker_pub = rospy.Publisher(
            "hss_marker", MarkerArray, queue_size=10)

        # Set up service proxies
        self.waypoint_push_srv = rospy.ServiceProxy(
            '/mavros/mission/push', WaypointPush)

        self.push_hss_coordinates = rospy.Service(
            "push_hss_coordinates", Trigger, self.push_hss_coordinates_callback)

        self.push_empty_fence = rospy.Service(
            "push_empty_fence", Trigger, self.push_empty_fence_callback)

        self.push_wp = rospy.ServiceProxy(
            '/mavros/geofence/push', WaypointPush)
        self.hss_coordinates = HavaSavunmaKoordinatlari()

    def hss_callback(self, data):
        self.hss_coordinates = data

    def draw_empty_fences(self):
        coordinates = [[36.942314, 35.563323], [36.942673, 35.553363], [
            36.937683, 35.553324], [36.937864, 35.562873]]
        fence_points = WaypointPushRequest()
        for index, coordinate in enumerate(coordinates):
            fence_point = Waypoint()
            fence_point.frame = 3  # GLOBAL (relative altitude)
            fence_point.command = 5001  # NAV_FENCE_POINT
            fence_point.is_current = False  # Not a current waypoint
            fence_point.autocontinue = True  # Autocontinue
            fence_point.param1 = 4
            fence_point.param2 = index
            fence_point.x_lat = coordinate[0]  # Latitude
            fence_point.y_long = coordinate[1]  # Longitude
            fence_point.z_alt = 100
            fence_points.waypoints.append(fence_point)
        return fence_points

    def push_hss_coordinates_callback(self, req):

        rospy.wait_for_service("/mavros/geofence/push")

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "hss"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker_array = MarkerArray()
        wp_push_request = self.draw_empty_fences()
        for data in self.hss_coordinates.hss_Koordinati:
            hssEnlem = data.hssEnlem  # Access fields from each item
            hssBoylam = data.hssBoylam
            hssYaricap = data.hssYaricap

            fence_point = Waypoint()
            fence_point.frame = 3  # GLOBAL (relative altitude)
            fence_point.command = 5004  # NAV_FENCE_POINT
            fence_point.is_current = False  # Not a current waypoint
            fence_point.autocontinue = True  # Autocontinue
            fence_point.param1 = hssYaricap  # For circular fences
            fence_point.x_lat = hssEnlem  # Latitude
            fence_point.y_long = hssBoylam  # Longitude
            fence_point.z_alt = 100  # Altitude in meters

            wp_push_request.waypoints.append(fence_point)

            # Conversion from geodetic to ENU
            x, y, z = pm.geodetic2enu(
                hssEnlem, hssBoylam, 0, self.HOME_LATITUDE, self.HOME_LONGITUDE, self.HOME_ALTITUDE)

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
            # Half of the height (since height is centered)
            marker.pose.position.z = 500

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
        response = self.push_wp(wp_push_request)
        if response.success:
            rospy.loginfo("Fence point pushed successfully!")
            return TriggerResponse(success=1)
        else:
            rospy.logwarn("Failed to push fence point.")
            return TriggerResponse(success=0)

    def push_empty_fence_callback(self, _):
        wp_list = self.draw_empty_fences()
        rospy.wait_for_service("/mavros/geofence/push")

        response = self.push_wp(wp_list)

        if response.success:
            rospy.loginfo("Fence point pushed successfully!")
            return TriggerResponse(success=1)
        else:
            rospy.logwarn("Failed to push fence point.")
            return TriggerResponse(success=0)

        """
        Calculate the great-circle distance between two points on the Earth's surface.
        """
        latitude_1_rad = math.radians(latitude_1)
        latitude_2_rad = math.radians(latitude_2)

        longitude_1_rad = math.radians(longitude_1)
        longitude_2_rad = math.radians(longitude_2)

        lat_difference = abs(latitude_1_rad - latitude_2_rad)
        lon_difference = abs(longitude_1_rad - longitude_2_rad)

        a = (math.sin(lat_difference / 2)) ** 2 + (math.cos(latitude_1_rad)
                                                   * math.cos(latitude_2_rad) * (math.sin(lon_difference / 2)) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = 6371000 * c
        return distance


if __name__ == '__main__':
    try:
        rospy.init_node("air_defense_node", anonymous=True)
        Air_Defense_Node()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt: {e}")
