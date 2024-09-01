import math
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf_trans
from geometry_msgs.msg import Quaternion
import rospy

EARTH_RADIUS = 6371000
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
    quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)
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

def calculate_waypoint(latitude, longitude, distance, bearing):
    """
    Calculate a new waypoint given a starting coordinate, distance, and bearing.
    """
    latitude_rad = math.radians(latitude)
    longitude_rad = math.radians(longitude)
    bearing_rad = math.radians(bearing)

    latitude_dest_rad = math.asin(
        math.sin(latitude_rad) * math.cos(distance / EARTH_RADIUS) +
        math.cos(latitude_rad) * math.sin(distance / EARTH_RADIUS) * math.cos(bearing_rad)
    )

    longitude_dest_rad = longitude_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(distance / EARTH_RADIUS) * math.cos(latitude_rad),
        math.cos(distance / EARTH_RADIUS) - math.sin(latitude_rad) * math.sin(latitude_dest_rad)
    )

    latitude_dest = math.degrees(latitude_dest_rad)
    longitude_dest = math.degrees(longitude_dest_rad)

    return latitude_dest, longitude_dest

def calculate_waypoints_with_angle(target_lat, target_lon, angle_degrees, distance1, distance2, distance3):
    R = 6371000  # Radius of the Earth
    try:
        # Convert latitude and longitude from degrees to radians
        target_lat_rad = math.radians(target_lat)
        target_lon_rad = math.radians(target_lon)
        angle_rad = math.radians(angle_degrees)

        # Calculate the waypoint at a certain distance and angle
        def calculate_waypoint(lat_rad, lon_rad, distance, angle):
            lat_new_rad = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                                    math.cos(lat_rad) * math.sin(distance / R) * math.cos(angle))
            lon_new_rad = lon_rad + math.atan2(math.sin(angle) * math.sin(distance / R) * math.cos(lat_rad),
                                               math.cos(distance / R) - math.sin(lat_rad) * math.sin(lat_new_rad))
            return lat_new_rad, lon_new_rad

        # Calculate waypoints
        wp1_lat_rad, wp1_lon_rad = calculate_waypoint(target_lat_rad, target_lon_rad, distance1, angle_rad)
        wp2_lat_rad, wp2_lon_rad = calculate_waypoint(target_lat_rad, target_lon_rad, distance2, angle_rad)
        wp5_lat_rad, wp5_lon_rad = calculate_waypoint(target_lat_rad, target_lon_rad, distance3, angle_rad + math.pi)  # Adding pi to get the opposite direction

        # Convert back to degrees
        wp1_lat_deg, wp1_lon_deg = math.degrees(wp1_lat_rad), math.degrees(wp1_lon_rad)
        wp2_lat_deg, wp2_lon_deg = math.degrees(wp2_lat_rad), math.degrees(wp2_lon_rad)
        wp5_lat_deg, wp5_lon_deg = math.degrees(wp5_lat_rad), math.degrees(wp5_lon_rad)

        return [(wp1_lat_deg, wp1_lon_deg),
                (wp2_lat_deg, wp2_lon_deg),
                (target_lat, target_lon),  # Target waypoint is just the target
                (wp5_lat_deg, wp5_lon_deg)]

    except Exception as e:
        rospy.logerr(f"Error calculating waypoints: {e}")
        return []  # Ensure it returns an empty list on error