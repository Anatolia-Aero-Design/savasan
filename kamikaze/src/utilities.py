import math
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf_trans
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
import rospy
from pyproj import CRS, Transformer,Proj,transform

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

def geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """
    Convert geodetic coordinates to ENU coordinates relative to a reference point.
    """
    # Define the CRS for the reference point
    crs_wgs84 = CRS.from_epsg(4326)  # WGS84 geodetic coordinates
    crs_utm = CRS(proj='utm', zone=33, datum='WGS84')  # Example UTM zone, adjust as necessary

    # Create a transformer for converting from geodetic to UTM
    transformer_geodetic_to_utm = Transformer.from_crs(crs_wgs84, crs_utm, always_xy=True)

    # Convert the reference point (lat, lon, alt) to UTM coordinates
    ref_utm = transformer_geodetic_to_utm.transform(ref_lon, ref_lat, ref_alt)

    # Convert the input point (lat, lon, alt) to UTM coordinates
    point_utm = transformer_geodetic_to_utm.transform(lon, lat, alt)

    # Calculate ENU coordinates
    e = point_utm[0] - ref_utm[0]
    n = point_utm[1] - ref_utm[1]
    u = point_utm[2] - ref_utm[2]

    return (e, n, u)

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

def calculate_coordinates(radius, azimuth_angle):
    if 0 < azimuth_angle <= 90:
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

def exit_calculate_coordinates(radius, azimuth_angle):
    if 0 < azimuth_angle <= 90:
        new_azimuth_angle_rad = math.radians(azimuth_angle)
        y = -math.cos(new_azimuth_angle_rad) * radius
        x = -math.sin(new_azimuth_angle_rad) * radius
    elif 90 < azimuth_angle <= 180:
        new_azimuth_angle_rad = math.radians(180 - azimuth_angle)
        y =  math.cos(new_azimuth_angle_rad) * radius
        x = -math.sin(new_azimuth_angle_rad) * radius
    elif 180 < azimuth_angle <= 270:
        new_azimuth_angle_rad = math.radians(270 - azimuth_angle)
        x = math.cos(new_azimuth_angle_rad) * radius
        y = math.sin(new_azimuth_angle_rad) * radius
    elif 270 < azimuth_angle <= 360:
        new_azimuth_angle_rad = math.radians(360 - azimuth_angle)
        y = -math.cos(new_azimuth_angle_rad) * radius
        x =  math.sin(new_azimuth_angle_rad) * radius
    if abs(x) < 1e-10:  
        x = 0

    return x, y

def lat_lon_to_ecef(lat, lon, alt):
    # WGS84 ellipsoid parameters
    a = 6378137.0  # semi-major axis in meters
    e = 8.181919e-2  # first eccentricity squared

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    N = a / math.sqrt(1 - e * math.sin(lat_rad) ** 2)
    x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
    z = ((1 - e) * N + alt) * math.sin(lat_rad)

    return x, y, z

def ecef_to_lat_lon_alt(x, y, z):
    """Convert ECEF coordinates to latitude, longitude, and altitude."""
    # WGS84 ellipsoid parameters
    a = 6378137.0  # Semi-major axis
    e2 = 0.00669437999014  # Eccentricity squared

    lon = np.arctan2(y, x)
    p = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p * (1 - e2))
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    alt = p / np.cos(lat) - N

    lat = np.degrees(lat)
    lon = np.degrees(lon)
    return lat, lon, alt

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

def calculate_waypoints_with_angle(target_lat, target_lon, angle_degrees, distance1, distance2, distance3,distance4):
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
        wp3_lat_rad, wp3_lon_rad = calculate_waypoint(target_lat_rad, target_lon_rad, distance3, angle_rad)
        wp2_lat_rad, wp2_lon_rad = calculate_waypoint(wp3_lat_rad, wp3_lon_rad, distance2, angle_rad)
        wp1_lat_rad, wp1_lon_rad = calculate_waypoint(wp2_lat_rad, wp2_lon_rad, distance1, angle_rad)
        wp5_lat_rad, wp5_lon_rad = calculate_waypoint(target_lat_rad, target_lon_rad, distance4, angle_rad + math.pi)  # Adding pi to get the opposite direction

        # Convert back to degrees
        wp1_lat_deg, wp1_lon_deg = math.degrees(wp1_lat_rad), math.degrees(wp1_lon_rad)
        wp2_lat_deg, wp2_lon_deg = math.degrees(wp2_lat_rad), math.degrees(wp2_lon_rad)
        wp3_lat_deg, wp3_lon_deg = math.degrees(wp3_lat_rad), math.degrees(wp3_lon_rad)
        wp5_lat_deg, wp5_lon_deg = math.degrees(wp5_lat_rad), math.degrees(wp5_lon_rad)

        return [(wp1_lat_deg, wp1_lon_deg),
                (wp2_lat_deg, wp2_lon_deg),
                (wp3_lat_deg, wp3_lon_deg),
                (wp5_lat_deg, wp5_lon_deg)]

    except Exception as e:
        rospy.logerr(f"Error calculating waypoints: {e}")
        return []  # Ensure it returns an empty list on error


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