#!/usr/bin/env python

from numpy import NaN
import numpy as np
import math
import rospy
from mavros_msgs.msg import AttitudeTarget,HomePosition
from mavros_msgs.srv import WaypointClear, SetMode,CommandInt,CommandBool
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import utilities as utils
from geometry_msgs.msg import PoseStamped, Vector3
import pymap3d as pm
from QR_node import QR_Node
from datetime import datetime 

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class AttitudeController:
    def __init__(self):
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.rate = rospy.Rate(20)  # 10 Hz

    def set_attitude(self, roll, pitch, yaw, thrust):
        attitude = AttitudeTarget()
        attitude.orientation = utils.euler_to_quaternion(roll, pitch, yaw)
        attitude.thrust = thrust
        self.attitude_pub.publish(attitude)

class Autonom_Maneuver_Node:
    def __init__(self) -> None:
        self.prev_time = rospy.get_time()
        
        self.qr_detected = False
        self.qr_start_initialized = False

        self.latitude = None
        self.longitude = None
        self.altitude = None

        self.qr_info = None
        self.x, self.y, self.z = None, None, None
        self.vector = None

        """self.HOME_LATITUDE = 36.9377789
        self.HOME_LONGITUDE = 35.5353955
        self.HOME_ALTITUDE = 100"""
        
        self.TARGET_LATITUDE = 36.93809669
        self.TARGET_LONGITUDE = 35.53314180 
        self.TARGET_ALTITUDE = 100.0
        
        self.PITCH_CORRECTION_ALTITUDE = 0.0 
        self.home_pose = None
        
        self.climb_waypoint = {
            'latitude': None,
            'longitude': None,
            'altitude': 100 # Default to home altitude
        }

        self.attitude_controller = AttitudeController()

        self.roll_pid = PIDController(kp=1.0, ki=0.4, kd=0.05)
        self.pitch_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)

        self.start_service = rospy.Service('start_kamikaze', Empty, self.start_waypoint)
        self.abort_service = rospy.Service('stop_kamikaze', Empty, self.stop_mission)
        self.abort_service = rospy.Service('abort_kamikaze', Empty, self.abort_mission)

        # Set mode service
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)

        # Initialize Subscribers
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.quaternion_callback)
        self.vector_sub = rospy.Subscriber('/uav_to_target_vector', Vector3, self.vector_callback)
        self.subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        self.rel_altitude_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.rel_altitude_callback)
        self.home_sub = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_callback)

        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        self.callback_active = False
        self.command_sent = False 

    def home_callback(self, data):
        self.home_pose = np.array([data.geo.latitude, data.geo.longitude, data.geo.altitude])
        self.HOME_LATITUDE = self.home_pose[0]
        self.HOME_LONGITUDE = self.home_pose[1]
        self.HOME_ALTITUDE = self.home_pose[2]
        return self.HOME_ALTITUDE,self.HOME_LONGITUDE,self.HOME_LATITUDE
        
    def quaternion_callback(self, data):
        self.x, self.y, self.z = utils.quaternion_to_euler_degrees(data.pose.orientation)
        
    def vector_callback(self, data):
        self.vector = data    

    def position_callback(self, data):
        # Extract data from the message
        self.latitude = float(data.latitude)
        self.longitude = float(data.longitude)
        
    def rel_altitude_callback(self, data):
        self.altitude = float(data.data) # Extract data from the message

    def callback(self):
        if self.latitude is None or self.longitude is None or self.altitude is None:
            rospy.logwarn("Current position data is incomplete.")
            return
        self.distance = utils.haversine_formula(self.latitude, self.longitude, self.TARGET_LATITUDE, self.TARGET_LONGITUDE)

        current_time = rospy.get_time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        if self.distance >= 100 and not self.command_sent:
            # Send position command to target
             self.send_position_command(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
             self.command_sent = True
             self.correct_heading()
             
    def get_current_time(self):
        now = datetime.now()
        return now.hour, now.minute, now.second, now.microsecond
                       
    def correct_heading(self):
        while True:
            distance = utils.haversine_formula(self.latitude, self.longitude, self.TARGET_LATITUDE, self.TARGET_LONGITUDE)
            
            roll_error, pitch_error = self.calculate_errors()
            current_time = rospy.get_time()
            dt = current_time - self.prev_time
            self.prev_time = current_time
            
            rospy.loginfo(f"vector:{self.vector}")
            self.qr_detected = self.qr_reader.read_check()
            if self.qr_detected:
                self.qr_info = True
                rospy.loginfo("QR code detected!")
            else:
                rospy.loginfo("QR code not detected.")
            
            if distance <= 150 and self.altitude > 50:
                rospy.loginfo(f"PITCH: {self.y}")
                rospy.loginfo(f"ROLL: {self.x}")
                rospy.loginfo(f"DISTANCE: {distance}")
                
                if self.qr_start_initialized is False:
                    self.start_time = self.get_current_time()
                    rospy.set_param('/kamikazeBaslangicZamani', self.start_time)
                    self.qr_start_initialized = True
                 
                self.perform_correction(roll_error,pitch_error, dt)
            
                rospy.loginfo(f"Roll error: {roll_error}")
                rospy.loginfo(f"Pitch error: {pitch_error}")
                rospy.loginfo(f"ALTITUDE: {self.altitude}")
            #elif distance <= 50 and self.altitude > 40:
            #   self.perform_dive_maneuver(roll_error, -35, dt,distance)
                
            elif self.qr_info or self.altitude <= 50:
                self.end_time = self.get_current_time()
                rospy.set_param('/kamikazeBitisZamani', self.end_time)
                self.perform_climb_maneuver()
                break
            rospy.sleep(1)

    def start_waypoint(self, req):
        
        self.qr_reader = QR_Node()
        if self.home_pose is None:
            rospy.logwarn("Home pose not available yet.")
            return EmptyResponse()
        try:
            rospy.loginfo("Waypoint calculations started.")
            
            rospy.wait_for_service('/mavros/set_mode')
            try:
                response = self.set_mode_srv(custom_mode='GUIDED')
                if response.mode_sent:
                    rospy.loginfo("Mode set to GUIDED")
                else:
                    rospy.logerr("Failed to set mode to GUIDED")
                    return EmptyResponse()
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                return EmptyResponse()

            rospy.wait_for_service('/mavros/cmd/arming')
            try:
                response = self.arm_srv(True)
                if response.success:
                    rospy.loginfo("Plane armed")
                else:
                    rospy.logerr("Failed to arm the plane")
                    return EmptyResponse()
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                return EmptyResponse()
            
            self.callback()
            self.callback_active = True
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return EmptyResponse()
        
    def abort_mission(self, req):
        self.perform_climb_maneuver()
        
    def stop_mission(self, req):
        try:
            # Stop any ongoing mission
            self.callback_active = False
            rospy.loginfo("Aborting mission.")
            self.qr_start_initialized = False
        
            # Clear existing waypoints
            rospy.wait_for_service('/mavros/mission/clear')
            try:
                clear_wp_srv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
                response = clear_wp_srv()
                if response.success:
                    rospy.loginfo("Cleared previous waypoints")
                else:
                    rospy.logwarn("Failed to clear waypoints")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to clear waypoints: {e}")
                return EmptyResponse()

            # Set mode to LOITER
            rospy.wait_for_service('/mavros/set_mode')
            try:
                set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                response = set_mode_srv(custom_mode='LOITER')
                if response.mode_sent:
                    rospy.loginfo("Mode set to LOITER")
                else:
                    rospy.logwarn("Failed to set mode to LOITER")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return EmptyResponse()

            self.command_sent = False
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
                                           param1 = 30, param2 = 0, param3 = 30, param4 = NaN, x = x, y = y, z = z)
            if response.success:
                rospy.loginfo(f"Current position: ({self.latitude}, {self.longitude}, {self.altitude})")
                rospy.loginfo(f"Command sent to: ({latitude}, {longitude}, {altitude})")
            else:
                rospy.logwarn(f"Failed to send command to ({latitude}, {longitude}, {altitude})")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            
    def perform_correction(self, roll_error,pitch_error, dt):
        """Perform correction based on error."""
        # Clear existing waypoints
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            clear_wp_srv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            response = clear_wp_srv()
            if response.success:
                rospy.loginfo("Cleared previous waypoints")
            else:
                rospy.logwarn("Failed to clear waypoints")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to clear waypoints: {e}")
            return
        
       # self.send_position_command(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, 0.0)
        roll_correction = self.roll_pid.compute(roll_error, dt)
        pitch_correction = self.pitch_pid.compute(pitch_error, dt)
        pitch = -(math.radians(pitch_correction))*2
        roll = (math.radians(roll_correction))/100
        self.attitude_controller.set_attitude(roll-1, pitch, 0, 0.5)
        rospy.loginfo(f"roll correction: {roll-1},")
        rospy.loginfo(f"pitch correction: {pitch},")
        return roll
    
    """def perform_dive_maneuver(self,roll_error,pitch,dt,distance):
        
         # Clear existing waypoints
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            clear_wp_srv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            clear_wp_srv()
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to clear waypoints: {e}")
            return
        
        rospy.loginfo(f"Performing DİVE")
        roll_correction = self.roll_pid.compute(roll_error, dt)
        roll = (math.radians(roll_correction))/10
        self.attitude_controller.set_attitude(roll-0.6, pitch, 0, 0.5)
        #rospy.loginfo(f"pitch correction: {pitch}")
        rospy.loginfo(f"roll correction: {roll-0.6}")

        rospy.loginfo(f"DİSTANCE: {distance}")
        rospy.loginfo(f"ALTİTUDE: {self.altitude}")
        rospy.loginfo(f"ROLL: {self.x},")
        rospy.loginfo(f"PİTCH {self.y}")  """
    
    def perform_climb_maneuver(self):
        rospy.loginfo("Entering climb maneuver")
        stop_service = rospy.ServiceProxy('stop_kamikaze', Empty)
        response = stop_service()
        
        # Set mode to RTL
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_srv(custom_mode='RTL')
            if response.mode_sent:
                rospy.loginfo("Mode set to RTL")
            else:
                rospy.logwarn("Failed to set mode to RTL")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return EmptyResponse()

        # Ensure all necessary data is available
        if self.vector is None:
            rospy.logwarn("Vector data is not available for climb maneuver.")
            return

        if self.latitude is None or self.longitude is None or self.altitude is None:
            rospy.logwarn("Current position data is incomplete.")
            return
    
        rospy.logwarn("perform climb manuever")
        # Define climb altitude and calculate the new target position
        target_altitude = 100  # Example: climb to 50 meters above home altitude

        # Set attitude for climb maneuver
        yaw = self.y
        roll = 0.0
        pitch = -20.0  # Adjust pitch for climb
        thrust = 0.7  # Adjust thrust for climb

        while self.altitude < target_altitude:
            rospy.loginfo(f"Current altitude: {self.altitude}. Climbing to {target_altitude} meters.")
            
            self.attitude_controller.set_attitude(roll, pitch, yaw, thrust)
            
            rospy.sleep(1)  # Wait before checking altitude again

        rospy.loginfo(f"Reached target altitude: {target_altitude} meters. Exiting climb maneuver.")
        
    def calculate_errors(self):
        
        if self.latitude is None or self.longitude is None:
            rospy.logwarn("Latitude and longitude data are not available yet.")
            return 0

        current_roll = self.x
        # Convert the current position and target position to ENU coordinates
        current_enu = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.HOME_LATITUDE,  self.HOME_LONGITUDE, self.HOME_ALTITUDE)
        target_enu = pm.geodetic2enu(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE, self.HOME_LATITUDE,  self.HOME_LONGITUDE, self.HOME_ALTITUDE)
        
        # Calculate the vector difference between the current position and target position
        vector_x = target_enu[0] - current_enu[0]
        vector_y = target_enu[1] - current_enu[1]
        
        # Compute the target roll using atan2
        target_roll = math.degrees(math.atan2(vector_y, vector_x))
        roll_error = (target_roll - current_roll)/100
        
        
        # Ensure self.vector is not None
        if self.vector is None:
            rospy.logwarn("Vector data is not available yet.")
            return 0, 0

        # Calculate the roll error
        """
        target_roll = math.degrees(math.atan2(self.vector.y, self.vector.x))
        roll_error = target_roll - current_roll"""

        # Calculate the pitch error
        current_pitch = self.y
        
        target_altitude_for_pitch = self.PITCH_CORRECTION_ALTITUDE
        altitude_difference = target_altitude_for_pitch - self.altitude
        
        distance_2d = math.sqrt(self.vector.x**2 + self.vector.y**2)
        target_pitch = math.degrees(math.atan2(altitude_difference, distance_2d))
        pitch_error = target_pitch - current_pitch
        return roll_error, pitch_error
  
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
    rospy.init_node('autonom_maneuver_node', anonymous=True)
    reposition = Autonom_Maneuver_Node()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed during node initialization {e}")