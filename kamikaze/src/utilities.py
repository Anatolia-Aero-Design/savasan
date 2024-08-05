import math
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf_trans
from geometry_msgs.msg import Quaternion

# Constant for Earth's radius in meters
EARTH_RADIUS = 6371000

def haversine_formula(latitude_1, longitude_1, latitude_2, longitude_2):
    """
    Calculate the great-circle distance between two points on the Earth's surface.
    """
    latitude_1_rad = math.radians(latitude_1)
    latitude_2_rad = math.radians(latitude_2)
    
    longitude_1_rad = math.radians(longitude_1)
    longitude_2_rad = math.radians(longitude_2)
    
    lat_difference = abs(latitude_1_rad - latitude_2_rad)
    lon_difference = abs(longitude_1_rad - longitude_2_rad)
    
    a = (math.sin(lat_difference / 2)) ** 2 + (math.cos(latitude_1_rad) * math.cos(latitude_2_rad) * (math.sin(lon_difference / 2)) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS * c
    return distance

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to a quaternion.
    """
    quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*quaternion)

def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion to Euler angles (yaw, pitch, roll) in degrees.
    """
    quat = [w, x, y, z]  # Quaternion order w, x, y, z
    r = R.from_quat(quat)
    euler = r.as_euler('zyx', degrees=True)  # Order: yaw, pitch, roll
    return euler[0], euler[1], euler[2]

def quaternion_to_euler_degrees(quaternion):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw) in degrees.
    """
    euler_angles_rad = tf_trans.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    euler_angles_deg = [math.degrees(angle) for angle in euler_angles_rad]
    return euler_angles_deg

def calculate_bearing(latitude_1, longitude_1, latitude_2, longitude_2):
    """
    Calculate the bearing between two geographic coordinates.
    """
    latitude_1_rad = math.radians(latitude_1)
    latitude_2_rad = math.radians(latitude_2)
    longitude_1_rad = math.radians(longitude_1)
    longitude_2_rad = math.radians(longitude_2)
    
    delta_long = longitude_2_rad - longitude_1_rad
    
    x = math.sin(delta_long) * math.cos(latitude_2_rad)
    y = math.cos(latitude_1_rad) * math.sin(latitude_2_rad) - (math.sin(latitude_1_rad) * math.cos(latitude_2_rad) * math.cos(delta_long))
    
    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = (bearing_deg + 360) % 360  # Normalize to [0, 360)
    
    return bearing_deg

def calculate_waypoint_sequence(target_lat, target_lon, target_alt, home_alt,
                                angle_degrees, distance1, distance2, distance3):
    """
    Calculate waypoints around a target coordinate at specified distances and angles.
    """
    target_lat_rad = math.radians(target_lat)
    target_lon_rad = math.radians(target_lon)

    # Calculate the waypoints
    wp1_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos((2 * distance1) / EARTH_RADIUS) +
                            math.cos(target_lat_rad) * math.sin((2 * distance1) / EARTH_RADIUS) * math.cos(math.radians(angle_degrees)))
    wp1_lon_rad = target_lon_rad + math.atan2(math.sin(math.radians(angle_degrees)) * math.sin((2 * distance1) / EARTH_RADIUS) * math.cos(target_lat_rad),
                            math.cos((2 * distance1) / EARTH_RADIUS) - math.sin(target_lat_rad) * math.sin(wp1_lat_rad))

    wp2_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(distance2 / EARTH_RADIUS) +
                            math.cos(target_lat_rad) * math.sin(distance2 / EARTH_RADIUS) * math.cos(math.radians(angle_degrees)))
    wp2_lon_rad = target_lon_rad + math.atan2(math.sin(math.radians(angle_degrees)) * math.sin(distance2 / EARTH_RADIUS) * math.cos(target_lat_rad),
                            math.cos(distance2 / EARTH_RADIUS) - math.sin(target_lat_rad) * math.sin(wp2_lat_rad))

    wp4_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(distance3 / EARTH_RADIUS) +
                            math.cos(target_lat_rad) * math.sin(distance3 / EARTH_RADIUS) * math.cos(math.radians(angle_degrees + 180)))
    wp4_lon_rad = target_lon_rad + math.atan2(math.cos(math.radians(angle_degrees + 90)) * math.sin(distance3 / EARTH_RADIUS) * math.cos(target_lat_rad),
                            math.cos(distance3 / EARTH_RADIUS) - math.sin(target_lat_rad) * math.sin(wp4_lat_rad))

    return [
        (math.degrees(wp1_lat_rad), math.degrees(wp1_lon_rad), home_alt),
        (math.degrees(wp2_lat_rad), math.degrees(wp2_lon_rad), home_alt),
        (target_lat, target_lon, target_alt),
        (math.degrees(wp4_lat_rad), math.degrees(wp4_lon_rad), home_alt)
    ]

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

def vector_angle(a, b):
    """
    Calculate the angle between two vectors in 3D space.
    """
    dot_product = sum(a[i] * b[i] for i in range(len(a)))
    mag_a = math.sqrt(sum(a[i] ** 2 for i in range(len(a))))
    mag_b = math.sqrt(sum(b[i] ** 2 for i in range(len(b))))
    cos_theta = dot_product / (mag_a * mag_b)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

def vector_angle_2d(a, b):
    """
    Calculate the angle between two vectors in 2D space.
    """
    dot_product = a[0] * b[0] + a[1] * b[1]
    mag_a = math.sqrt(a[0] ** 2 + a[1] ** 2)
    mag_b = math.sqrt(b[0] ** 2 + b[1] ** 2)
    cos_theta = dot_product / (mag_a * mag_b)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)
    return angle_deg
