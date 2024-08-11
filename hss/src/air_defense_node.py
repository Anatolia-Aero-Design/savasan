#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandInt, SetMode
from mavros_msgs.msg import CommandCode
from std_srvs.srv import Empty, EmptyResponse
import requests
import numpy as np
import logging
from mavros_msgs.srv import WaypointPush
import kamikaze.src.utilities as utils

class Air_Defense_Node:
    def __init__(self) -> None:
        self.uav_pose_sub = None
        
        self.uav_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.uav_pose_callback)
        self.server_url_hss_koordinatlari = rospy.get_param('/comm_node/api/hss_koordinatlari')
        self.waypoint_push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        self.geofence_list = []
        
        self.start_service = rospy.Service('enable_geo_fences', Empty, self.enable_geo_fences)
        self.stop_service = rospy.Service('disable_geo_fences', Empty, self.disable_geo_fences)
    
    def uav_pose_callback(self, data):
        self.uav_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    
    def enable_geo_fences(self, req):
        try:
            self.parse_coordinates()
            self.send_geofences()
            self.take_action_RTL()
            rospy.loginfo("Geofences enabled!")
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return EmptyResponse()
        
    def disable_geo_fences(self, req):
        try:    
            self.send_command(CommandCode.DO_FENCE_ENABLE, 0)
            self.geofence_list = []
            rospy.loginfo("Geofences disabled!")
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return EmptyResponse()
        
    def take_action_RTL(self):
        distance_list = []
        for i, item in enumerate(self.geofence_list):
            distance = utils.haversine_formula(self.uav_position[0], self.uav_position[1], item[0], item[1])
            distance -= item[2]
            distance_list.append(distance)
        for distance in distance_list:
            if distance <= 0:
                self.set_mode("RTL")
            
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
                    0, 0, 0, 0, 
                )
            
            # Complete the geofence definition
            self.send_command(CommandCode.DO_FENCE_ENABLE, 0)
        except Exception as e:
            rospy.logerr(f"Unexpected error occurred: {e}")
    
    def send_command(self, command, param1, param2=2, param3=0, param4=0, x=0, y=0, z=0):
        try:
            rospy.wait_for_service('/mavros/cmd/command_int')
            response = self.command_int_srv(
                broadcast=False,
                frame=3,
                command=command,
                current=0,
                autocontinue=False, 
                param1=param1,
                param2=param2,
                param3=param3,
                param4=param4,
                x=x,
                y=y,
                z=z
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

if __name__ == '__main__':
    try:
        rospy.init_node("air_defense_node", anonymous=True)
        Air_Defense_Node()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt: {e}")

