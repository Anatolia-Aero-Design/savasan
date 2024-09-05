import math
from scipy.spatial.transform import Rotation as R
from mavros_msgs.srv import ParamSetRequest,ParamSet,ParamGet,ParamGetRequest
from mavros_msgs.msg import ParamValue
import tf.transformations as tf_trans
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer,Proj,transform
import rospy


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

def geodetic_to_cartesian(lat, lon, alt):
    # WGS84 ellipsoid parameters
    a = 6378137.0  # Semi-major axis in meters
    e = 8.1819190842622e-2  # Eccentricity
    
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    
    N = a / np.sqrt(1 - e**2 * np.sin(lat_rad)**2)
    
    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1 - e**2) + alt) * np.sin(lat_rad)
    
    return np.array([x, y, z])

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to a quaternion.
    """
    quaternion = tf_trans.quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
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

def calculate_waypoint_sequence(current_lat, current_lon, current_alt, target_lat, target_lon, 
                                target_alt, azimuth_angle, distance_1_2, dive_angle, distance_leave):
    """
    Generate waypoints based on the given target location, azimuth angle, and distances for a dive maneuver.

    Args:
    - current_lat (float): Current latitude of the plane (waypoint 2).
    - current_lon (float): Current longitude of the plane (waypoint 2).
    - current_alt (float): Current altitude of the plane (waypoint 2).
    - target_lat (float): Latitude of the target location (waypoint 3).
    - target_lon (float): Longitude of the target location (waypoint 3).
    - target_alt (float): Altitude of the target location (waypoint 3).
    - azimuth_angle (float): Azimuth angle in degrees (0-360) from the North.
    - distance_1_2 (float): Distance between waypoint 1 and waypoint 2 (in meters).
    - dive_angle (float): Dive angle in degrees (between waypoint 2 and waypoint 3).
    - distance_leave (float): Distance between waypoint 3 and waypoint 4 (in meters).
    
    Returns:
    - list of dicts: A list of waypoints with their latitude, longitude, and altitude.
    """
    
    # Convert angles from degrees to radians
    azimuth_rad = math.radians(azimuth_angle)
    dive_rad = math.radians(dive_angle)

    # Calculate the change in latitude/longitude for distance 1 -> 2
    delta_lat_1_2 = (distance_1_2 / EARTH_RADIUS) * math.cos(azimuth_rad)
    delta_lon_1_2 = (distance_1_2 / (EARTH_RADIUS * math.cos(math.radians(current_lat)))) * math.sin(azimuth_rad)

    # Waypoint 1: Starting point (before the dive)
    waypoint_1_lat = current_lat - math.degrees(delta_lat_1_2)
    waypoint_1_lon = current_lon - math.degrees(delta_lon_1_2)
    waypoint_1_alt = current_alt  # Altitude remains the same as the plane's current altitude

    # Calculate the distance between waypoint 2 and 3 based on the dive angle
    distance_2_3 = (current_alt - target_alt) / math.tan(dive_rad)

    # Calculate the change in latitude/longitude for distance 2 -> 3
    delta_lat_2_3 = (distance_2_3 / EARTH_RADIUS) * math.cos(azimuth_rad)
    delta_lon_2_3 = (distance_2_3 / (EARTH_RADIUS * math.cos(math.radians(target_lat)))) * math.sin(azimuth_rad)

    # Waypoint 2: The point before the dive with the current altitude
    waypoint_2_lat = waypoint_1_lat + math.degrees(delta_lat_2_3)
    waypoint_2_lon = waypoint_1_lon + math.degrees(delta_lon_2_3)
    waypoint_2_alt = current_alt  # Plane's current altitude
    
    # Waypoint 3: The target itself (end of the dive)
    waypoint_3_lat = target_lat
    waypoint_3_lon = target_lon
    waypoint_3_alt = target_alt  # Target altitude

    # Calculate the change in latitude/longitude for distance leaving the target
    delta_lat_leave = (distance_leave / EARTH_RADIUS) * math.cos(azimuth_rad)
    delta_lon_leave = (distance_leave / (EARTH_RADIUS * math.cos(math.radians(target_lat)))) * math.sin(azimuth_rad)

    # Waypoint 4: Leaving point after the target
    waypoint_4_lat = target_lat + math.degrees(delta_lat_leave)
    waypoint_4_lon = target_lon + math.degrees(delta_lon_leave)
    waypoint_4_alt = target_alt  # Altitude remains the same as the target altitude

    waypoints = [
        [waypoint_1_lat, waypoint_1_lon, waypoint_1_alt],
        [waypoint_2_lat, waypoint_2_lon, waypoint_2_alt],
        [waypoint_3_lat, waypoint_3_lon, waypoint_3_alt],
        [waypoint_4_lat, waypoint_4_lon, waypoint_4_alt],
    ]
    return waypoints

def enu_to_geodetic(east, north, up, origin_lat, origin_lon, origin_alt):
    # Define the projection system for ENU
    transformer = Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)
    
    # Convert origin from geodetic to ECEF
    origin_ecef = transformer.transform(origin_lon, origin_lat, origin_alt)

    # Use the ENU to ECEF conversion equations
    # For simplicity, we'll assume a flat Earth approximation here.
    lat = origin_lat + north / 111000  # Approximate conversion (1 degree latitude ~ 111 km)
    lon = origin_lon + east / (111000 * math.cos(math.radians(origin_lat)))  # Approximate conversion
    alt = origin_alt + up

    return lat, lon, alt

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
   
"""
This part of the module is containing image processing functions for qr detection 
"""

def adjust_brightness_contrast(image, brightness=0, contrast=0):
    # Brightness: -255 to 255, Contrast: -127 to 127
    if brightness != 0:
        image = cv2.convertScaleAbs(image, beta=brightness)
    if contrast != 0:
        image = cv2.addWeighted(image, 1.0 + contrast / 127.0, image, 0, 0)
    return image

def gamma_correction(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([(i / 255.0) ** invGamma * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def convert_to_grayscale(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

def adaptive_threshold(image):
    gray = convert_to_grayscale(image)
    return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

def histogram_equalization(image):
    gray = convert_to_grayscale(image)
    return cv2.equalizeHist(gray)

def color_filter(image, lower_bound, upper_bound):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    return cv2.bitwise_and(image, image, mask=mask)

def edge_detection(image):
    gray = convert_to_grayscale(image)
    edges = cv2.Canny(gray, 100, 200)
    return edges

def noise_reduction(image):
    return cv2.GaussianBlur(image, (5, 5), 0)

def adjust_exposure(image, exposure=-1.0):
    # Exposure: negative for less exposure, positive for more
    return cv2.convertScaleAbs(image, alpha=1, beta=exposure)

def reduce_glare(image):
    # Apply Gaussian Blur and thresholding to reduce glare
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    return cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]

def print_param():
    param_list = ['PTCH_LIM_MIN_DEG', 'TECS_SINK_MAX']
    
    try:
        service_proxy = rospy.ServiceProxy('/mavros/param/get', ParamGet)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    
    for param in param_list:
        try:
            request = ParamGetRequest()
            request.param_id = param 
            response = service_proxy(request)
            if response.success:
                print(f"Parameter {param} has value: {response.value.real if response.value.real != 0 else response.value.integer}")
            else:
                rospy.logwarn(f"Failed to get parameter {param}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def set_params_for_dive():
    rospy.wait_for_service('/mavros/param/set/')
    param_list = ['PTCH_LIM_MIN_DEG','TECS_SINK_MAX']
    value = [-50,20]
    try:
        service_proxy = rospy.ServiceProxy('/mavros/param/set/', ParamSet)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    for index, param in enumerate(param_list):
        try:
            request = ParamSetRequest()
            request.param_id = param 
            request.value = ParamValue(integer=value[index])
            response = service_proxy(request)
            rospy.loginfo(f"Service response: {response}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        

def set_params_for_safe_fly():
    rospy.wait_for_service('/mavros/param/set/')
    param_list = ['PTCH_LIM_MIN_DEG','TECS_SINK_MAX']
    value = [-20,5]
    try:
        service_proxy = rospy.ServiceProxy('/mavros/param/set/', ParamSet)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    for index, param in enumerate(param_list):
        try:
            request = ParamSetRequest()
            request.param_id = param 
            request.value = ParamValue(integer=value[index])
            response = service_proxy(request)
            rospy.loginfo(f"Service response: {response}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


def calculate_coordinates(radius, azimuth_angle):

    if 0 <= azimuth_angle <= 90:
        new_azimuth_angle_rad = math.radians(azimuth_angle)
        y = math.cos(new_azimuth_angle_rad) * radius
        x = math.sin(new_azimuth_angle_rad) * radius
    elif 90 < azimuth_angle <= 180:
        new_azimuth_angle_rad = math.radians(180 - azimuth_angle)
        y = -math.cos(new_azimuth_angle_rad) * radius
        x =  math.sin(new_azimuth_angle_rad) * radius
    elif 180 < azimuth_angle <= 270:
        new_azimuth_angle_rad = math.radians(270 - azimuth_angle)
        x = -math.cos(new_azimuth_angle_rad) * radius
        y = -math.sin(new_azimuth_angle_rad) * radius
    elif 270 < azimuth_angle <= 360:
        new_azimuth_angle_rad = math.radians(360 - azimuth_angle)
        y = math.cos(new_azimuth_angle_rad) * radius
        x = -math.sin(new_azimuth_angle_rad) * radius
    if abs(x) < 1e-10:  
        x = 0

    return x, y