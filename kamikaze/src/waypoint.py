#!/usr/bin/env python

import dis
from numpy import NaN
import rospy
import math
from mavros_msgs.srv import CommandInt, SetMode
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
import utilities as utils
from visualization_msgs.msg import Marker

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

class WaypointNode:
    def __init__(self) -> None:
        
        # Initialize variables
        self.latitude = None
        self.longitude = None
        self.altitude = None
        
        self.qr_info = None
        self.angle_degrees = 70
        self.quaternion = None
        self.x, self.y, self.z = None, None, None
        self.vector = None
        
        # Home coordinates
        self.HOME_LATITUDE = -35.363212
        self.HOME_LONGITUDE = 149.165210
        self.HOME_ALTITUDE = 100
        
        # Target coordinates
        self.TARGET_LATITUDE = -35.360849
        self.TARGET_LONGITUDE = 149.161835
        self.TARGET_ALTITUDE = 10 # small positive value for safety purposes (plane denies value of 0)
        
        self.attitude_controller = AttitudeController()
        
        # Initialize PID controllers for roll and pitch
        self.roll_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.pitch_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.prev_time = rospy.get_time()
        
        # Initialize services
        self.start_service = rospy.Service('start_waypoint_calculations', Empty, self.start_waypoint)
        self.abort_service = rospy.Service('abort_kamikaze_mission', Empty, self.abort_mission)

        # Set mode service
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
        
        # Initialize Subscribers
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.quaternion_callback)
        self.vector_sub = rospy.Subscriber('/uav_to_target_vector', Vector3, self.vector_callback)
        self.subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        self.rel_altitude_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.rel_altitude_callback)
        
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
        
    def timer_callback(self, event):
        self.callback()

    def callback(self):
        if self.latitude is None or self.longitude is None or self.altitude is None:
            return
        
        distance = utils.haversine_formula(self.latitude, self.longitude, self.TARGET_LATITUDE, self.TARGET_LONGITUDE)
        rospy.loginfo(f"Distance to target: {distance} meters")
        
        roll_error, pitch_error = self.calculate_errors()

        current_time = rospy.get_time()
        dt = current_time - self.prev_time
        self.prev_time = current_time        
        
        if distance >= 100:
            self.set_mode("GUIDED")
            self.send_position_command(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
        
        elif 50 < distance <= 100:
            roll = self.perform_roll_correction(roll_error, dt)
            rospy.loginfo(f"Roll error: {roll_error}")
        elif distance <= 50:
            pitch = self.perform_pitch_correction(pitch_error, dt)
            rospy.loginfo(f"Pitch error: {pitch_error}")

            self.perform_dive_maneuver(0, pitch)
            if self.qr_info is not None or self.altitude <= 30:
                self.perform_climb_maneuver(-20)
    
    def start_waypoint(self, req):
        try:
            rospy.loginfo("Waypoint calculations started.")
            # Timer for constant distance calculation and corrections
            self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return EmptyResponse()

    def abort_mission(self, req):
        ...

    def send_position_command(self, latitude, longitude, altitude):
        try:
            rospy.wait_for_service('/mavros/cmd/command_int')

            x = int(latitude * 1e7)
            y = int(longitude * 1e7)
            z = altitude

            response = self.command_int_srv(broadcast = False, frame = 3, command = 192, current = 0, autocontinue = False, 
                                           param1 = 30, param2 = 0, param3 = 30, param4 = NaN, x = x, y = y, z = z)

            if response.success:
                rospy.loginfo(f"Current position: ({self.latitude}, {self.longitude}, {self.altitude})")
                rospy.loginfo(f"Command sent to: ({latitude}, {longitude}, {altitude})")
            else:
                rospy.logwarn(f"Failed to send command to ({latitude}, {longitude}, {altitude})")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
    def perform_roll_correction(self, roll_error, dt):
        """Perform roll correction based on error."""
        roll_correction = self.roll_pid.compute(roll_error, dt)
        roll = math.radians(roll_correction)
        pitch = 0
        yaw = 0
        thrust = 0.5
        rospy.loginfo(f"Performing roll correction with error: {roll_error:.2f} degrees")
        self.attitude_controller.set_attitude(roll, pitch, yaw, thrust)
        return roll

    def perform_pitch_correction(self, pitch_error, dt):
        """Perform pitch correction based on error."""
        pitch_correction = self.pitch_pid.compute(pitch_error, dt)
        pitch = math.radians(pitch_correction)
        yaw = 0
        roll = 0.0
        thrust = 0.5
        rospy.loginfo(f"Performing pitch correction with error: {pitch_error:.2f} degrees")
        self.attitude_controller.set_attitude(roll, pitch, yaw, thrust)
        return pitch

    def perform_dive_maneuver(self, yaw, pitch):
        roll = 0.0
        thrust = 0.5
        self.attitude_controller.set_attitude(roll, pitch, yaw, thrust)
    
    def perform_climb_maneuver(self, pitch_deg):
        yaw = 0
        roll = 0.0
        pitch = math.radians(pitch_deg)
        thrust = 0.5
        self.attitude_controller.set_attitude(roll, pitch, yaw, thrust)
        
    def calculate_errors(self):
        if self.vector is None:
            rospy.logwarn("Vector data is not available yet.")
            return 0, 0

        current_roll = self.x
        target_roll = math.degrees(math.atan2(self.vector.y, self.vector.x))
        roll_error = target_roll - current_roll
        roll_error = ((roll_error + 180) % 360) - 180
        rospy.loginfo(f"Roll error: {roll_error}")

        current_pitch = self.y
        distance_2d = math.sqrt(self.vector.x**2 + self.vector.y**2)
        target_pitch = math.degrees(math.atan2(self.vector.z, distance_2d))
        pitch_error = target_pitch - current_pitch
        rospy.loginfo(f"Pitch error: {pitch_error}")

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
    rospy.init_node('waypoint_node', anonymous=True)
    WaypointNode()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed during node initialization {e}")