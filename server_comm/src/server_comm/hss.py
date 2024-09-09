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

# Plane does not support NAV_FENCE_CIRCLE_EXCLUSION

class Air_Defense_Node:
    def __init__(self,fence_list) -> None:
        # Initialize variables
        self.uav_pose_sub = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.state = None
        self.heading = None
        self.geofence_list = fence_list
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("hss")
        self.mission_file_path = os.path.join(package_path, "mission_folder/fence-items.txt")

        # Set up service proxies
        self.waypoint_push_srv = rospy.ServiceProxy(
            '/mavros/mission/push', WaypointPush)
        

    def enable_geo_fences(self):
        try:
            self.write_mission_to_file()
            self.send_command(CommandCode.DO_FENCE_ENABLE, 1)
            rospy.loginfo("Geofences enabled!")
            return TriggerResponse(success=True)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return TriggerResponse(success=False)

    def disable_geo_fences(self):
        try:
            # Clear the fence-items.txt file
            with open(self.mission_file_path, 'w') as file:
                pass  # Open in write mode and close immediately to clear the file
            self.send_command(CommandCode.DO_FENCE_ENABLE, 0)
            self.geofence_list = []
            rospy.loginfo("Geofences disabled!")
            return TriggerResponse(success=True)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return TriggerResponse(success=False)

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
            rospy.loginfo(
                f"Mission file written successfully to {self.mission_file_path}")
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