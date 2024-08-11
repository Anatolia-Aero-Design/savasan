#!/usr/bin/env python

from numpy import NaN
import rospy
import math
from mavros_msgs.srv import CommandInt, SetMode
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
import utilities as utils

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
        self.rate = rospy.Rate(10)  # 10 Hz

    def set_attitude(self, roll, pitch, yaw, thrust):
        attitude = AttitudeTarget()
        attitude.orientation = utils.euler_to_quaternion(roll, pitch, yaw)
        attitude.thrust = thrust
        self.attitude_pub.publish(attitude)

class AutonomNode:
    def __init__(self) -> None:
        # Initialize variables
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.qr_info = None
        self.x, self.y, self.z = None, None, None
        self.vector = None
        
        # Home coordinates
        self.HOME_LATITUDE  = 36.9377791
        self.HOME_LONGITUDE = 35.5353955
        self.HOME_ALTITUDE  = 100
        
        # Target coordinates
        self.TARGET_LATITUDE  = 36.938828
        self.TARGET_LONGITUDE = 35.532133
        self.TARGET_ALTITUDE  = 100  # Small positive value for safety purposes (plane denies value of 0)
        
        self.attitude_controller = AttitudeController()
        
        # Initialize PID controllers for roll and pitch
        self.roll_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.pitch_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.prev_time = rospy.get_time()

        self.start_service = rospy.Service('start_waypoint_calculations', Empty, self.start_waypoint)
        self.abort_service = rospy.Service('abort_kamikaze_mission', Empty, self.abort_mission)
        
        # Set mode service
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
        
        # Initialize Subscribers
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.quaternion_callback)
        self.vector_sub = rospy.Subscriber('/uav_to_target_vector', Vector3, self.vector_callback)
        self.global_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        self.rel_altitude_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.rel_altitude_callback)
    
    def start_waypoint(self, req):
        try:
            rospy.loginfo("Waypoint calculations started.")
            self.callback()
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return EmptyResponse()

    def abort_mission(self, req):
        rospy.logwarn("Mission aborted.")
        return EmptyResponse()
    
    def quaternion_callback(self, data):
        self.x, self.y, self.z = utils.quaternion_to_euler_degrees(data.pose.orientation)
        
    def vector_callback(self, data):
        self.vector = data
        #rospy.loginfo(f"Updated vector: x={self.vector.x}, y={self.vector.y}, z={self.vector.z}")
    
    def position_callback(self, data):
        self.latitude = float(data.latitude)
        self.longitude = float(data.longitude)
        
    def rel_altitude_callback(self, data):
        self.altitude = float(data.data)
        
    def callback(self):
        if self.latitude is None or self.longitude is None or self.altitude is None:
            return

        self.distance = utils.haversine_formula(self.latitude, self.longitude, self.TARGET_LATITUDE, self.TARGET_LONGITUDE)

        if self.distance >= 100:
            self.set_mode("GUIDED")
            self.send_position_command(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
            self.correct_heading()
    
    def correct_heading(self):
        while True:
            self.distance = utils.haversine_formula(self.latitude, self.longitude, self.TARGET_LATITUDE, self.TARGET_LONGITUDE)
            rospy.loginfo(f"Distance to target: {self.distance} meters")
            
            roll_error, pitch_error = self.calculate_errors()
            current_time = rospy.get_time()
            dt = current_time - self.prev_time
            self.prev_time = current_time
            
            if 50 < self.distance <= 100:
                self.perform_roll_correction(roll_error, dt)
                rospy.loginfo(f"Roll error: {roll_error}")
            if self.distance <= 50:
                self.perform_pitch_correction(pitch_error, dt)
                rospy.loginfo(f"Pitch error: {pitch_error}")
                self.perform_dive_maneuver(0, pitch_error)
                if self.qr_info or self.altitude <= 30:
                    self.perform_climb_maneuver(-20)
                    break
            rospy.sleep(1)
    
    def send_position_command(self, latitude, longitude, altitude):
        try:
            rospy.wait_for_service('/mavros/cmd/command_int')
            x = int(latitude * 1e7)
            y = int(longitude * 1e7)
            z = altitude
            response = self.command_int_srv(broadcast=False, frame=3, command=192, current=0, autocontinue=False, 
                                            param1=30, param2=0, param3=30, param4=NaN, x=x, y=y, z=z)
            if response.success:
                rospy.loginfo(f"Current position: ({self.latitude}, {self.longitude}, {self.altitude})")
                rospy.loginfo(f"Command sent to: ({latitude}, {longitude}, {altitude})")
            else:
                rospy.logwarn(f"Failed to send command to ({latitude}, {longitude}, {altitude})")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def perform_roll_correction(self, roll_error, dt):
        roll_correction = self.roll_pid.compute(roll_error, dt)
        roll = math.radians(roll_correction)
        self.attitude_controller.set_attitude(roll, 0, 0, 0.5)
        return roll

    def perform_pitch_correction(self, pitch_error, dt):
        pitch_correction = self.pitch_pid.compute(pitch_error, dt)
        pitch = math.radians(pitch_correction)
        self.attitude_controller.set_attitude(0, pitch, 0, 0.5)
        return pitch

    def perform_dive_maneuver(self, yaw, pitch):
        self.attitude_controller.set_attitude(0, pitch, yaw, 0.5)
    
    def perform_climb_maneuver(self, pitch_deg):
        pitch = math.radians(pitch_deg)
        self.attitude_controller.set_attitude(0, pitch, 0, 0.5)

    def calculate_errors(self):
        if self.vector is None:
            rospy.logwarn("Vector data is not available yet.")
            return 0, 0

        current_roll = self.x
        target_roll = math.degrees(math.atan2(self.vector.y, self.vector.x))
        # target_roll = ((target_roll +90) % 180) - 90
        print(target_roll)
        roll_error = target_roll - current_roll
        roll_error = ((roll_error + 90) % 180) - 90

        current_pitch = self.y
        distance_2d = math.sqrt((self.altitude)**2 + self.distance**2)
        target_pitch = math.degrees(math.atan2(self.vector.z, distance_2d))
        pitch_error = target_pitch - current_pitch

        return roll_error, pitch_error

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
        # Initialize node
        rospy.init_node('autonom_node', anonymous=True)
        autonom_node = AutonomNode()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt: {e}")
