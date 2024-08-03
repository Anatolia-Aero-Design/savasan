#!/usr/bin/env python

from numpy import NaN
import rospy
from mavros_msgs.srv import CommandInt, SetMode
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
import utilities as utils
import requests

class WaypointNode:
    def __init__(self) -> None:
        
        # Initialize variables
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.qr_info = None
        self.angle_degrees = 70
        self.calculated_waypoints = None 
        self.waypoint_reached = False
        
        # Home coordinates
        self.HOME_LATITUDE = -35.363212
        self.HOME_LONGITUDE = 149.165210
        self.HOME_ALTITUDE = 100
        
        # Target coordinates
        self.TARGET_LATITUDE = None
        self.TARGET_LONGITUDE = None
        self.TARGET_ALTITUDE = 15.0

        # API URLs
        self.server_url_qr_koordinati = rospy.get_param('/comm_node/api/qr_koordinati')
        
        # Initialize services
        self.start_service = rospy.Service('start_waypoint_calculations', Empty, self.start_waypoint)
        self.abort_service = rospy.Service('abort_kamikaze_mission', Empty, self.abort_mission)

        # Set mode service
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
        
        # Subscribe to the GLOBAL_POSITION_INT topic

        self.subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        self.rel_altitude_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.rel_altitude_callback)

    def position_callback(self, data):
        # Extract data from the message
        self.latitude = float(data.latitude)
        self.longitude = float(data.longitude)
        
    def rel_altitude_callback(self, data):
        self.altitude = float(data.data) # Extract data from the message

    def get_coordinates(self):
        try:
            response = requests.get(self.server_url_qr_koordinati)
            if response.status_code == 200:
                json_data = response.json()
                self.TARGET_LATITUDE = json_data.get("qrEnlem")
                self.TARGET_LONGITUDE = json_data.get("qrBoylam")
                rospy.loginfo(f"QR Latitude: {self.TARGET_LATITUDE}, QR Longitude: {self.TARGET_LONGITUDE}")
            else:
                rospy.logwarn(f"Error: Unable to fetch data from URL. Status code: {response.status_code}")
        except Exception as e:
            rospy.logerr(f"Error: {e}")
            
    def start_waypoint(self, req):
        try:
            self.get_coordinates()
            rospy.loginfo("Waypoint calculations started.")
            self.callback()
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return EmptyResponse()

    def abort_mission(self, req):
        try:
            # Abort the mission by sending the last calculated waypoint
            if self.calculated_waypoints:
                self.send_position_command(self.calculated_waypoints[3][0], self.calculated_waypoints[3][1], self.HOME_ALTITUDE)
            rospy.loginfo("Aborting mission.")
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return EmptyResponse()

    def send_position_command(self, latitude, longitude, altitude):
        try:
            rospy.wait_for_service('/mavros/cmd/command_int')

            # Convert latitude and longitude to the required format (1e7)
            x = int(latitude * 1e7)
            y = int(longitude * 1e7)
            z = altitude

            # Create the CommandInt request
            response = self.command_int_srv(broadcast = False, frame = 3, command = 192, current = 0, autocontinue = False, 
                                           param1 = 0, param2 = 0, param3 = 30, param4 = NaN, x = x, y = y, z = z)

            if response.success:
                rospy.loginfo(f"Current position: ({self.latitude}, {self.longitude}, {self.altitude})")
                rospy.loginfo(f"Command sent to: ({latitude}, {longitude}, {altitude})")
            else:
                rospy.logwarn(f"Failed to send command to ({latitude}, {longitude}, {altitude})")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def callback(self):
        self.set_mode("GUIDED")
        rospy.sleep(2)  # Ensure the mode change has taken effect

        self.send_position_command(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
        rospy.loginfo(f"Heading to target position")
        distance = utils.haversine_formula(self.latitude, self.longitude, self.TARGET_LATITUDE, self.TARGET_LONGITUDE)
        if distance <= 100:
            ...
        
    def calculate_vectors(self):
        ...
            
            
    def set_mode(self, mode):
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=5)
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_service(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Mode set to {mode}")
            else:
                rospy.logwarn(f"Failed to set mode to {mode}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")        


if __name__ == '__main__':
    rospy.init_node('waypoint_node', anonymous=True)
    WaypointNode()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed during node initialization {e}")