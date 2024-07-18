#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear
import math
import requests
import logging

class Waypoint_node:
    def __init__(self) -> None:
        self.mission_wps = False 
        self.HOME_LATITUDE = -35.363212
        self.HOME_LONGITUDE = 149.165210
        self.HOME_ALTITUDE = 100.0
        
        self.ROAD_DISTANCE = 300  
        self.TARGET_LATITUDE = self.TARGET_LATITUDE
        self.TARGET_LONGITUDE = self.TARGET_LONGITUDE
        self.TARGET_ALTITUDE = 15.0
        self.server_url_qr_koordinati = rospy.get_param('/comm_node/api/qr_koordinati')
        
    def get_coordinates(self):
        try:
            response = requests.get(self.server_url_qr_koordinati)
            print(f"Response Status Code: {response.status_code}")
            
            if response.status_code == 200:
                json_data = response.json()
                print("JSON Data:")
            else:
                print(f"Error: Unable to fetch data from URL. Status code: {response.status_code}")
        except Exception as e:
            print(f"Error: {e}")
            
    def calculate_waypoints(self, target_lat, target_lon):
        R = 6371000  # radi of the earth

        target_lat_rad = math.radians(target_lat) #latitude and longitude from degrees to radians
        target_lon_rad = math.radians(target_lon)

        delta_lon = math.atan2(self.ROAD_DISTANCE, R * math.cos(target_lat_rad)) # calculate the distance in radians for distance between wps
    
        wp1_lat = math.degrees(target_lat_rad)
        wp1_lon = math.degrees(target_lon_rad - 2 * delta_lon)

        wp2_lat = math.degrees(target_lat_rad)
        wp2_lon = math.degrees(target_lon_rad - delta_lon)

        wp5_lat = target_lat
        wp5_lon = target_lon + math.degrees(delta_lon)  # Adjusted longitude

        return [(wp1_lat, wp1_lon), (wp2_lat, wp2_lon), (target_lat, target_lon), (wp5_lat, wp5_lon)]

    def push_waypoints(self, waypoints):
        rospy.wait_for_service('/plane1/mavros/mission/push')
        clear_waypoints_service = rospy.ServiceProxy('/plane1/mavros/mission/clear', WaypointClear)
        push_waypoints_service = rospy.ServiceProxy('/plane1/mavros/mission/push', WaypointPush)
        try:
            response = clear_waypoints_service()
            response = push_waypoints_service(0, waypoints.waypoints)  
            return response.wp_transfered
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False

    def waypoints(self):
        if self.mission_wps:
            waypoints = WaypointList()

            wp1 = Waypoint()
            wp1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp1.command = 16
            wp1.is_current = True  
            wp1.autocontinue = True
            wp1.param1 = 0
            wp1.param2 = 0
            wp1.param3 = 0
            wp1.x_lat = self.HOME_LATITUDE
            wp1.y_long = self.HOME_LONGITUDE
            wp1.z_alt = self.HOME_ALTITUDE
            waypoints.waypoints.append(wp1)
            
            calculated_waypoints = self.calculate_waypoints(self.TARGET_LATITUDE, self.TARGET_LONGITUDE)
            
            wp2 = Waypoint()
            wp2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp2.command = 16
            wp2.is_current = False  
            wp2.autocontinue = True
            wp2.param1 = 0
            wp2.param2 = 0
            wp2.param3 = 0
            wp2.x_lat = calculated_waypoints[0][0]
            wp2.y_long = calculated_waypoints[0][1]
            wp2.z_alt = self.HOME_ALTITUDE
            waypoints.waypoints.append(wp2)

            wp3 = Waypoint()
            wp3.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp3.command = 16
            wp3.is_current = False  
            wp3.autocontinue = True
            wp3.param1 = 0
            wp3.param2 = 0
            wp3.param3 = 0
            wp3.x_lat = calculated_waypoints[1][0]
            wp3.y_long = calculated_waypoints[1][1]
            wp3.z_alt = self.HOME_ALTITUDE
            waypoints.waypoints.append(wp3)

            wp4 = Waypoint()
            wp4.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp4.command = 16
            wp4.is_current = False  
            wp4.autocontinue = True
            wp4.param1 = 0
            wp4.param2 = 0
            wp4.param3 = 0
            wp4.x_lat  = self.TARGET_LATITUDE
            wp4.y_long = self.TARGET_LONGITUDE
            wp4.z_alt  = self.TARGET_ALTITUDE
            waypoints.waypoints.append(wp4)

            wp5 = Waypoint()
            wp5.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp5.command = 16
            wp5.is_current = False 
            wp5.autocontinue = True
            wp5.param1 = 0
            wp5.param2 = 0
            wp5.param3 = 0
            wp5.x_lat = calculated_waypoints[3][0]
            wp5.y_long = calculated_waypoints[3][1]
            wp5.z_alt = self.HOME_ALTITUDE
            waypoints.waypoints.append(wp5)

            success = self.push_waypoints(waypoints)
            if success:
                rospy.loginfo("Waypoints pushed successfully!")
            else:
                rospy.logerr("Failed to push waypoints!")
                
        else:
            rospy.loginfo("Parameter is false")

    def mission_wps_callback(self, data):
        self.mission_wps = data.data

if __name__ == "__main__":
    rospy.init_node('waypoint_node', anonymous=True)
    waypoint_node = Waypoint_node()
    rospy.loginfo("Waiting for mission_wps")
    try:
        rospy.spin()
    except Exception as e:
        logging.error(f"An error occurred in the main loop: {str(e)}")