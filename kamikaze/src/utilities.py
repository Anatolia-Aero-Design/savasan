import math
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf_trans
from geometry_msgs.msg import Quaternion
import rospy
from mavros_msgs.srv import CommandLong

def haversine_formula(latitude_1, longtitude_1, latitude_2, longtitude_2):
    R = 6371000 # Radius of the Earth in meters
    latitude_1_rad = math.radians(latitude_1)
    latitude_2_rad = math.radians(latitude_2)
    
    longtitude_1_rad = math.radians(longtitude_1)
    longtitude_2_rad = math.radians(longtitude_2)
    
    lat_difference = abs(latitude_1_rad - latitude_2_rad)
    lon_difference = abs(longtitude_1_rad - longtitude_2_rad)
    
    a = (math.sin(lat_difference/2))**2 + (math.cos(latitude_1_rad) * math.cos(latitude_2_rad) * (math.sin(lon_difference/2))**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf_trans.quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
    return Quaternion(*quaternion)

def quaternion_to_euler(w, x, y, z):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).
    Roll is rotation around x-axis (in degrees).
    Pitch is rotation around y-axis (in degrees).
    Yaw is rotation around z-axis (in degrees).
    
    :param w: Quaternion w component
    :param x: Quaternion x component
    :param y: Quaternion y component
    :param z: Quaternion z component
    :return: tuple (roll, pitch, yaw) in degrees
    """
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        # Use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert radians to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    return roll, pitch, yaw



def gps_to_xyz(home_lat, home_lon, home_alt, target_lat, target_lon, target_alt):
    """
    Converts a GPS location to an XYZ coordinate system relative to a home location.
    
    :param home_lat: Latitude of the home location in degrees
    :param home_lon: Longitude of the home location in degrees
    :param home_alt: Altitude of the home location in meters
    :param target_lat: Latitude of the target location in degrees
    :param target_lon: Longitude of the target location in degrees
    :param target_alt: Altitude of the target location in meters
    :return: A tuple (x, y, z) representing the target location in XYZ coordinates relative to the home location
    """
    # Constants
    earth_radius = 6378137.0  # Earth's radius in meters
    
    # Convert latitude and longitude from degrees to radians
    home_lat_rad = math.radians(home_lat)
    home_lon_rad = math.radians(home_lon)
    target_lat_rad = math.radians(target_lat)
    target_lon_rad = math.radians(target_lon)
    
    # Calculate differences in latitude, longitude, and altitude
    delta_lat = target_lat_rad - home_lat_rad
    delta_lon = target_lon_rad - home_lon_rad
    delta_alt = target_alt - home_alt
    
    # Calculate X and Y distances (East and North) in meters
    delta_y = delta_lat * earth_radius  # North direction (Y axis)
    delta_x = delta_lon * earth_radius * math.cos(home_lat_rad)  # East direction (X axis)
    
    # Z coordinate is simply the difference in altitude
    delta_z = delta_alt
    
    return delta_x, delta_y, delta_z


def send_command_long(target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7):
    """
    Send a COMMAND_LONG MAVLink message via MAVROS.

    :param target_system: Target system (e.g., drone) ID
    :param target_component: Target component ID (e.g., autopilot)
    :param command: MAVLink command ID
    :param confirmation: 0: First transmission of this command, 1-255: Confirmation transmissions (e.g. for kill command)
    :param param1 to param7: Command parameters (use 0 if not needed)
    :return: Success status and result of the command
    """

    # Wait for the service to be available
    rospy.wait_for_service('/mavros/cmd/command')

    try:
        # Create a service proxy for the command_long service
        command_long_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        # Call the service
        response = command_long_service(target_system,
                                        target_component,
                                        command,
                                        confirmation,
                                        param1,
                                        param2,
                                        param3,
                                        param4,
                                        param5,
                                        param6,
                                        param7)

        # Check the response and return the result
        if response.success:
            rospy.loginfo(f"Command {command} sent successfully with result {response.result}")
        else:
            rospy.logwarn(f"Failed to send command {command} with result {response.result}")

        return response.success, response.result

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False, None





