#!/usr/bin/env python

import rospy
from rospy import ROSException
from mavros_msgs.srv import CommandTOL, CommandTOLResponse
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointClear
from std_srvs.srv import Empty, EmptyResponse
import requests
import logging

class Air_Defense_Node:
    def __init__(self) -> None:
        self.server_url_hss_koordinatlari = rospy.get_param('/comm_node/api/hss_koordinatlari')
        self.waypoint_push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.waypoint_clear_srv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)

        self.parsed_list = []
        self.geofence_list = []

        self.start_service = rospy.Service('enable_geo_fences', Empty, self.enable_geo_fences)
        
    def enable_geo_fences(self, req):
        try:
            self.parse_coordinates()
            #self.upload_geofences()
            rospy.loginfo("Geofences enabled!")
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return EmptyResponse()
    
    def get_coordinates(self):
        try:
            response = requests.get(self.server_url_hss_koordinatlari)
            if response.status_code == 200:
                coordinate_data = response.json().get('hss_koordinat_bilgileri')
                logging.info(f"Air defense coordinates retrieved successfully")
                return coordinate_data
            else:
                logging.error(f"Failed to retrieve air defense coordinates, status code: {response.status_code}")
                return None
        except Exception as e:
            logging.error(f"An error occurred while retrieving air defense coordinates: {str(e)}")
            return None
    
    def parse_coordinates(self):
        test_waypoints = [
            Waypoint(
                frame=3,
                command=16,
                is_current=False,
                autocontinue=True,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                x_lat=-35.360947,
                y_long=149.164313,
                z_alt=0
            ),
            Waypoint(
                frame=3,
                command=16,
                is_current=False,
                autocontinue=True,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                x_lat=-35.362358,
                y_long=149.167855,
                z_alt=0
            )
        ]
        try:
            rospy.loginfo("Uploading test waypoints...")
            response = self.waypoint_push_srv(waypoints=test_waypoints)
            if response.success:
                rospy.loginfo("Test waypoints uploaded successfully.")
            else:
                rospy.logwarn("Failed to upload test waypoints.")
        except ROSException as e:
            rospy.logerr(f"Unexpected error occurred: {e}")

    def upload_geofences(self):
        try:
            # Clear existing waypoints
            self.waypoint_clear_srv()
            
            # Upload new waypoints
            response = self.waypoint_push_srv(waypoints=self.geofence_list)
            if response.success:
                rospy.loginfo("Geofence waypoints uploaded successfully.")
            else:
                rospy.logwarn("Failed to upload geofence waypoints.")
        except Exception as e:
            rospy.logerr(f"Unexpected error occurred: {e}")

if __name__ == '__main__':
    try:
        rospy.init_node("air_defense_node", anonymous=True)
        Air_Defense_Node()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt: {e}")
