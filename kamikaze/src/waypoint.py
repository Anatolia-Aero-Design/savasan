#!/usr/bin/env python

import dis
from numpy import NaN
import math
import rospy
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import WaypointClear, SetMode,CommandInt,CommandBool
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
import utilities as utils
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3
import threading

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
        
        self.latitude = None
        self.longitude = None
        self.altitude = None

        self.qr_info = None
        self.angle_degrees = 70
        self.quaternion = None
        self.x, self.y, self.z = None, None, None
        self.vector = None


        self.HOME_LATITUDE = -35.363212
        self.HOME_LONGITUDE = 149.165210
        self.HOME_ALTITUDE = 100.0
        
        self.TARGET_LATITUDE = -35.360849
        self.TARGET_LONGITUDE = 149.161835
        self.TARGET_ALTITUDE = 20.0
        
        
        self.climb_waypoint = {
            'latitude': None,
            'longitude': None,
            'altitude': 100 # Default to home altitude
        }

        self.attitude_controller = AttitudeController()

        self.roll_pid = PIDController(kp=2.0, ki=0.0, kd=0.1)
        self.pitch_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.prev_time = rospy.get_time()

        self.start_service = rospy.Service('start_kamikaze', Empty, self.start_waypoint)
        self.abort_service = rospy.Service('stop_kamikaze', Empty, self.abort_mission)

        # Set mode service
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)

        # Initialize Subscribers
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.quaternion_callback)
        self.vector_sub = rospy.Subscriber('/uav_to_target_vector', Vector3, self.vector_callback)
        self.subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        self.rel_altitude_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.rel_altitude_callback)

        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        self.callback_active = False
        self.command_sent = False 
        
        self.callback_thread = threading.Thread(target=self.run_callback_continuously)
        self.callback_thread.daemon = True
        self.callback_thread.start()

        
        
    #def send_qr_info(self, msg):
    #    self.qr_info = msg
    #    if not None (self.qr_info):
    #        try:
    #            response = requests.post(self.server_url_kamikaze_bilgisi, json=self.qr_info)
    #            logging.info(f"Sent kamikaze data to server: {self.mission_wps}")
    #        except Exception as e:
    #            logging.error(f"Failed to send data, status code: {response.status_code}")
    
    def run_callback_continuously(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.callback_active:
                self.callback()
            rate.sleep()



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
        # Timer callback to repeatedly call the main logic
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

             
    def callback(self):
        if self.latitude is None or self.longitude is None or self.altitude is None:
            return
        distance = utils.haversine_formula(self.latitude, self.longitude, self.TARGET_LATITUDE, self.TARGET_LONGITUDE)
        

        current_time = rospy.get_time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        
        if distance >= 100 and not self.command_sent:
            # Send position command to target
             self.send_position_command(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
             self.command_sent = True
             
        while True:
            
            distance = utils.haversine_formula(self.latitude, self.longitude, self.TARGET_LATITUDE, self.TARGET_LONGITUDE)
            if distance <= 50:
                
                # Check conditions to perform climb maneuver
                if self.qr_info is not None or self.altitude <= 50:
                    self.perform_climb_maneuver()

            elif 50 < distance < 200:
                roll_error, pitch_error = self.calculate_errors()
                # If the distance is between 50 and 100 meters, just log the info
                # Perform dive maneuver if within 50 meters
                roll = self.perform_roll_correction(roll_error, dt)
                rospy.loginfo(f"Roll error: {roll_error}")
                self.perform_dive_maneuver(yaw=0, pitch=-70)
                rospy.loginfo(f"Approaching target, distance: {distance} meters")

    def start_waypoint(self, req):
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
            
            
            self.callback_active = True
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return EmptyResponse()



    def abort_mission(self, req):
        try:
            self.callback_active = False
            rospy.loginfo("Aborting mission.")
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
        roll = 0.0  # Maintain current roll
        thrust = 0.7  # Adjust as needed for your maneuver

        # Set the new target position for the dive
        dive_target_latitude = self.TARGET_LATITUDE  # You can adjust this as needed
        dive_target_longitude = self.TARGET_LONGITUDE  # You can adjust this as needed
        dive_target_altitude = self.altitude - 50  # Example: Dive 10 meters lower; adjust as needed

        # Perform the dive maneuver
        rospy.loginfo(f"Performing dive maneuver with pitch: {pitch}, thrust: {thrust}")
        self.attitude_controller.set_attitude(roll, pitch, yaw, thrust)
        
        # Send the position command to the new target position
        self.send_position_command(dive_target_latitude, dive_target_longitude, dive_target_altitude)

        rospy.loginfo(f"Dive maneuver set to new altitude: {dive_target_altitude} meters with thrust: {thrust}")
    
    def perform_climb_maneuver(self):
        rospy.loginfo("Entering climb maneuver")

        # Ensure all necessary data is available
        if self.vector is None:
            rospy.logwarn("Vector data is not available for climb maneuver.")
            return

        if self.latitude is None or self.longitude is None or self.altitude is None:
            rospy.logwarn("Current position data is incomplete.")
            return

        rospy.logwarn("perform climb manuever")
        # Define climb altitude and calculate the new target position
        target_altitude = self.HOME_ALTITUDE + 50  # Example: climb to 50 meters above home altitude
        distance_above = 1000  # This should be adjusted as needed
        current_heading = math.radians(self.y)

        # Compute new target coordinates
        new_latitude = self.latitude + (distance_above * math.cos(current_heading) / 1e7)
        new_longitude = self.longitude + (distance_above * math.sin(current_heading) / 1e7)

        # Update climb waypoint
        self.climb_waypoint['latitude'] = new_latitude
        self.climb_waypoint['longitude'] = new_longitude
        self.climb_waypoint['altitude'] = target_altitude

        # Set attitude for climb maneuver
        yaw = self.y
        roll = 0.0
        pitch = 20.0  # Adjust pitch for climb
        thrust = 0.7  # Adjust thrust for climb

        rospy.loginfo(f"Performing climb maneuver to new waypoint: ({new_latitude}, {new_longitude}, {target_altitude})")
        
        # Set attitude
        self.attitude_controller.set_attitude(roll, pitch, yaw, thrust)

        # Send the waypoint command
        self.send_position_command(new_latitude, new_longitude, target_altitude)
        
        rospy.loginfo(f"Climb maneuver set to new altitude: {target_altitude} meters with thrust: {thrust}")


        
    def calculate_errors(self):
        # Ensure self.vector is not None
        if self.vector is None:
            rospy.logwarn("Vector data is not available yet.")
            return 0, 0

        # Calculate the roll error
        current_roll = self.x
        target_roll = math.degrees(math.atan2(self.vector.y, self.vector.x))
        roll_error = target_roll - current_roll
        roll_error = ((roll_error + 180) % 360) - 180
        rospy.loginfo(f"Roll error: {roll_error}")

        # Calculate the pitch error
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
    reposition = WaypointNode()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed during node initialization {e}")