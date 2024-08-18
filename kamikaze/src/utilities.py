import math
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf_trans
from geometry_msgs.msg import Quaternion

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

def quaternion_to_euler(x, y, z, w):
    quat = [w, x, y, z]  # Quaternion sırası w, x, y, z
    r = R.from_quat(quat)
    euler = r.as_euler('zyx', degrees=True)  # Yaw, Pitch, Roll sırası
    return euler[0], euler[1], euler[2]

def quaternion_to_euler_degrees(quaternion):
    # Convert quaternion to Euler angles (roll, pitch, yaw) in radians
    euler_angles_rad = tf_trans.euler_from_quaternion((quaternion.x, quaternion.y,quaternion.z,quaternion.w))
    
    # Convert Euler angles from radians to degrees
    euler_angles_deg = [math.degrees(angle) for angle in euler_angles_rad]
    
    return euler_angles_deg



