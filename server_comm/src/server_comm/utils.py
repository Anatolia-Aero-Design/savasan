from scipy.spatial.transform import Rotation as R
from roslibpy import Message
import json
import math
from datetime import datetime, timezone


def calculate_speed(x, y):
    speed = math.sqrt(x**2 + y**2)
    return speed

def mode_guided(mode):
    if mode == True:
        return 0
    else: 
        return 1

def unix_to_utc_formatted(unix_time, nsecs):
    # Convert Unix epoch time to UTC datetime
    utc_time = datetime.fromtimestamp(unix_time, tz=timezone.utc)
    
    # Format the UTC time to include day, hour, minute, second, and milliseconds
    formatted_time = utc_time.strftime("%d:%H:%M:%S.%f")[:-3]
    formatted_time = {"saat": int(utc_time.strftime("%H")),
                      "dakika": int(utc_time.strftime("%M")),
                      "saniye": (int(utc_time.strftime("%S"))),
                      "milisaniye": int(nsecs/1000000)}

    return formatted_time

def quaternion_to_euler(x, y, z, w):
    quat = [w, x, y, z]  # Quaternion sırası w, x, y, z
    r = R.from_quat(quat)
    euler = r.as_euler('zyx', degrees=True)  # Yaw, Pitch, Roll sırası
    return euler[0], euler[1], euler[2]