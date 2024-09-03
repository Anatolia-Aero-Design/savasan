#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandInt, SetMode
from mavros_msgs.msg import CommandCode, State
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
import requests
import logging
from mavros_msgs.srv import WaypointPush
import math

## Plane does not support NAV_FENCE_CIRCLE_EXCLUSION but when given fence list calculations are made correctly and RTL mode enabled 

class Air_Defense_Node:
    def __init__(self) -> None:
        # Initialize variables
        self.uav_pose_sub = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.state = None
        self.heading = None
        self.geofence_list = []
        self.mission_file_path = "/home/poyrazzo/fence-items.txt"
        
        # Set up subscribers
        self.uav_pose_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.uav_pose_callback)
        self.rel_altitude_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.rel_altitude_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.heading_sub = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.heading_callback)
        
        # Get server url from sim.launch
        self.server_url_hss_koordinatlari = rospy.get_param('/comm_node/api/hss_koordinatlari')
        
        # Set up service proxies        
        self.waypoint_push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.start_service = rospy.Service('enable_geofences', Empty, self.enable_geo_fences)
        self.stop_service = rospy.Service('disable_geofences', Empty, self.disable_geo_fences)
        
        # Create a timer to periodically perform tasks
        self.timer = rospy.Timer(rospy.Duration(0.5), self.periodic_task)
    
    def heading_callback(self, data):
        self.heading = float(data.data)
        
    def uav_pose_callback(self, data):
        self.latitude = float(data.latitude)
        self.longitude = float(data.longitude)
    
    def rel_altitude_callback(self, data):
        self.altitude = float(data.data)
        
    def state_callback(self, msg):
        self.state = msg.mode
    
    def periodic_task(self, event):
        # Perform periodic tasks here, e.g., checking geofence status
        self.take_action_RTL()
        
    def enable_geo_fences(self, req):
        try:
            self.parse_coordinates()
            self.write_mission_to_file()
            self.take_action_RTL()
            self.send_command(CommandCode.DO_FENCE_ENABLE, 1)
            rospy.loginfo("Geofences enabled!")
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return EmptyResponse()
        
    def disable_geo_fences(self, req):
        try:    
            # Clear the fence-items.txt file
            with open(self.mission_file_path, 'w') as file:
                pass  # Open in write mode and close immediately to clear the file
            self.send_command(CommandCode.DO_FENCE_ENABLE, 0)
            self.geofence_list = []
            rospy.loginfo("Geofences disabled!")
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return EmptyResponse()
        
    def take_action_RTL(self):
        distance_list = []
        for item in self.geofence_list:
            distance = self.haversine_formula(self.latitude, self.longitude, item["hssEnlem"], item["hssBoylam"])
            distance -= item["hssYaricap"]
            distance_list.append(distance)
        for distance in distance_list:
            if distance <= 0 and self.state != 'RTL':
                self.set_mode("RTL")
                self.response = True
        
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
        data_dict = self.get_coordinates()
        if data_dict is None:
            rospy.logwarn("No data received from the coordinates server.")
            return
        
        self.geofence_list = []
        for item in data_dict:
            hssEnlem = item["hssEnlem"]
            hssBoylam = item["hssBoylam"]
            hssYaricap = item["hssYaricap"]
            self.geofence_list.append({
                "hssEnlem": hssEnlem,
                "hssBoylam": hssBoylam,
                "hssYaricap": hssYaricap
            })
        return self.geofence_list

    def create_fence_mission(self):
        mission_data = "QGC WPL 110\n"
        
        # Add each fence as a line in the mission file
        for i, fence in enumerate(self.geofence_list):
            line = f"{i}\t0\t0\t5004\t{fence['hssYaricap']}\t0\t0\t0\t{fence['hssEnlem']}\t{fence['hssBoylam']}\t0\t1"
            mission_data += line + "\n"
        
        return mission_data

    def write_mission_to_file(self):
        data = self.create_fence_mission()
        try:
            with open(self.mission_file_path, 'w') as file:
                file.write(data)
            rospy.loginfo(f"Mission file written successfully to {self.mission_file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to write mission file: {e}")

    def send_geofences(self):
        try:    
            # Enable geofence system
            self.send_command(CommandCode.DO_FENCE_ENABLE, 1)

            # Send each geofence circle exclusion
            for i, geofence in enumerate(self.geofence_list):
                # Sending a circular exclusion zone
                self.send_command(
                    CommandCode.NAV_FENCE_CIRCLE_EXCLUSION,
                    i,
                    geofence["hssYaricap"],  # Radius
                    geofence["hssEnlem"],    # Latitude
                    geofence["hssBoylam"],   # Longitude
                    0, 0, 0)
            
            # Complete the geofence definition
            self.send_command(CommandCode.DO_FENCE_ENABLE, 0)
        except Exception as e:
            rospy.logerr(f"Unexpected error occurred: {e}")
    
    def send_command(self, command, param1, param2=0, param3=0, param4=0, x=0, y=0, z=0):
        try:
            rospy.wait_for_service('/mavros/cmd/command_int')
            response = self.command_int_srv(
                broadcast=False,
                frame=3,
                command=command,
                current=0,
                autocontinue=0,  # Convert False to 0 (unsigned integer)
                param1=int(param1),
                param2=int(param2),
                param3=int(param3),
                param4=int(param4),
                x=int(x),
                y=int(y),
                z=int(z)
            )
            if response.success:
                rospy.loginfo(f"Command {command} sent to plane successfully")
            else:
                rospy.logwarn(f"Failed to send command {command}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_mode(self, mode):
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=5)
            response = self.set_mode_srv(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Mode set to {mode}")
            else:
                rospy.logwarn(f"Failed to set mode to {mode}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def haversine_formula(self, latitude_1, longitude_1, latitude_2, longitude_2):
        """
        Calculate the great-circle distance between two points on the Earth's surface.
        """
        latitude_1_rad = math.radians(latitude_1)
        latitude_2_rad = math.radians(latitude_2)
        
        longitude_1_rad = math.radians(longitude_1)
        longitude_2_rad = math.radians(longitude_2)
        
        lat_difference = abs(latitude_1_rad - latitude_2_rad)
        lon_difference = abs(longitude_1_rad - longitude_2_rad)
        
        a = (math.sin(lat_difference / 2)) ** 2 + (math.cos(latitude_1_rad) * math.cos(latitude_2_rad) * (math.sin(lon_difference / 2)) ** 2)
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
