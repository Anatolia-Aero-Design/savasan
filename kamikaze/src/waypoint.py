#!/usr/bin/env python

import time
import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32, String
import math
import requests
import logging

class Waypoint_node:
    def __init__(self) -> None:
        self.qr_info = None
        self.angle_degrees = int(input("Enter dive angle: ")) 
        self.HOME_LATITUDE = -35.363212
        self.HOME_LONGITUDE = 149.165210
        self.HOME_ALTITUDE = 100.0
        
        self.ROAD_DISTANCE = 300  
        self.TARGET_LATITUDE = -35.383212
        self.TARGET_LONGITUDE = 149.175210
        self.TARGET_ALTITUDE = 15.0
        self.server_url_qr_koordinati = rospy.get_param('/comm_node/api/qr_koordinati')
        self.server_url_kamikaze_bilgisi = rospy.get_param('/comm_node/api/kamikaze_bilgisi')
        
        self.qr_sub = rospy.Subscriber('/qr_code_data', String)
                
    #def send_qr_info(self, msg):
    #    self.qr_info = msg
    #    if not None (self.qr_info):
    #        try:
    #            response = requests.post(self.server_url_kamikaze_bilgisi, json=self.qr_info)
    #            logging.info(f"Sent kamikaze data to server: {self.mission_wps}")
    #        except Exception as e:
    #            logging.error(f"Failed to send data, status code: {response.status_code}")
             
    def get_coordinates(self):
        try:
            response = requests.get(self.server_url_qr_koordinati)
    
            if response.status_code == 200:
                json_data = response.json()
                self.TARGET_LATITUDE = json_data.get("qrEnlem")
                self.TARGET_LONGITUDE = json_data.get("qrBoylam")
                rospy.loginfo(f"QR Latitude: {self.TARGET_LATITUDE}, QR Longitude: {self.TARGET_LONGITUDE}")
                
            else:
                print(f"Error: Unable to fetch data from URL. Status code: {response.status_code}")
                
        except Exception as e:
            print(f"Error: {e}")
            
    def push_waypoints(self, waypoints):
        rospy.wait_for_service('/mavros/mission/push')
        clear_waypoints_service = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        push_waypoints_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        try:
            response = clear_waypoints_service()
            response = push_waypoints_service(0, waypoints.waypoints)  
            return response.wp_transfered
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False        
            
            
    def calculate_waypoints_with_angle(self, target_lat, target_lon, angle_degrees,distance1,distance2,distance3):
        R = 6371000  # Radius of the Earth

        # Convert latitude and longitude from degrees to radians
        target_lat_rad = math.radians(target_lat)
        target_lon_rad = math.radians(target_lon)

        # Calculate the first waypoint (left) with the specified angle
        wp1_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos((2 * distance1) / R) +
                        math.cos(target_lat_rad) * math.sin((2 * distance1) / R) * math.cos(math.radians(angle_degrees)))
        wp1_lon_rad = target_lon_rad + math.atan2(math.sin(math.radians(angle_degrees)) * math.sin((2 * distance1) / R) * math.cos(target_lat_rad),
                        math.cos((2 * distance1) / R) - math.sin(target_lat_rad) * math.sin(wp1_lat_rad))

        # Calculate the second waypoint (right) with the specified angle
        wp2_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(distance2 / R) +
                        math.cos(target_lat_rad) * math.sin(distance2 / R) * math.cos(math.radians(angle_degrees)))
        wp2_lon_rad = target_lon_rad + math.atan2(math.sin(math.radians(angle_degrees)) * math.sin(distance2 / R) * math.cos(target_lat_rad),
                        math.cos(distance2 / R) - math.sin(target_lat_rad) * math.sin(wp2_lat_rad))

        # Calculate the leaving waypoint (same latitude as target, different longitude)

        wp5_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(distance3 / R) +
                        math.cos(target_lat_rad) * math.sin(distance3 / R) * math.cos(math.radians(angle_degrees + 180)))
        wp5_lon_rad = target_lon_rad + math.atan2(math.cos(math.radians(angle_degrees + 90)) * math.sin(distance3 / R) * math.cos(target_lat_rad),
                        math.cos(distance3 / R) - math.sin(target_lat_rad) * math.sin(wp5_lat_rad))

        return [(math.degrees(wp1_lat_rad), math.degrees(wp1_lon_rad)),
                (math.degrees(wp2_lat_rad), math.degrees(wp2_lon_rad)),
                (math.degrees(target_lat_rad), math.degrees(target_lon_rad)),
                (math.degrees(wp5_lat_rad), math.degrees(wp5_lon_rad))]


    def waypoints(self):
        self.get_coordinates()
    
        # Define waypoints
        waypoints = WaypointList()

        # Waypoint 1: home position
        wp1 = Waypoint()
        wp1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp1.command = 16
        wp1.is_current = True  
        wp1.autocontinue = True
        wp1.param1 = 0
        wp1.param2 = 0
        wp1.param3 = 0
        wp1.x_lat  = self.HOME_LATITUDE
        wp1.y_long = self.HOME_LONGITUDE
        wp1.z_alt  = self.HOME_ALTITUDE
        waypoints.waypoints.append(wp1)

        # Calculate waypoints with the specified angle
        calculated_waypoints = self.calculate_waypoints_with_angle(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.angle_degrees, 250,250,250)

        # Existing code...
        # Waypoint 2: Road waypoint 1
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

        # Waypoint 3: Road waypoint 2
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

        # Waypoint 4: Target coordinates and target altitude
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

        # Waypoint 5: Leave the target coordinates
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

        # Push waypoints to the flight controller
        success = self.push_waypoints(waypoints)
        print(waypoints)
        if success:
            rospy.loginfo("Waypoints pushed successfully!")
        else:
            rospy.logerr("Failed to push waypoints!")

    def mission_wps_callback(self, data):
        self.mission_wps = data.data
        
if __name__ == '__main__':
    rospy.init_node('waypoint_node', anonymous=True)
    waypoints = Waypoint_node()
    waypoints.waypoints()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed during node initialization {e}")