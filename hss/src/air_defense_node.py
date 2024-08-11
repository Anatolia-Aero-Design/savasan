#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandInt
from mavros_msgs.msg import CommandCode
from std_srvs.srv import Empty, EmptyResponse
import requests
import logging
from mavros_msgs.srv import WaypointPush

class Air_Defense_Node:
    def __init__(self) -> None:
        self.server_url_hss_koordinatlari = rospy.get_param('/comm_node/api/hss_koordinatlari')
        self.waypoint_push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
        
        self.parsed_list = {}
        self.geofence_list = []
        
        self.start_service = rospy.Service('enable_geo_fences', Empty, self.enable_geo_fences)
        
    def enable_geo_fences(self, req):
        try:
            self.parse_coordinates()
            self.send_geofences()
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

if __name__ == '__main__':
    try:
        rospy.init_node("air_defense_node", anonymous=True)
        Air_Defense_Node()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt: {e}")

