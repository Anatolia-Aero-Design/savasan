#!/usr/bin/env python

import rospy
from server_comm.msg import KonumBilgileri, KonumBilgisi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations
from pyproj import CRS, Transformer
import math
import numpy as np


# WGS84 ellipsoid constants
a = 6378137.0         # Semi-major axis
f = 1 / 298.257223563 # Flattening
e2 = 2*f - f**2  

def geodetic_to_enu(lat, lon, alt):
    lat_ref =  36.93824690
    lon_ref = 35.52944677
    alt_ref = 0
    # Convert degrees to radians
    lat_ref = np.deg2rad(lat_ref)
    lon_ref = np.deg2rad(lon_ref)
    lat = np.deg2rad(lat)
    lon = np.deg2rad(lon)

    # Calculate prime vertical radius of curvature
    N = a / np.sqrt(1 - e2 * np.sin(lat_ref)**2)

    # Calculate ECEF coordinates of reference point
    x_ref = (N + alt_ref) * np.cos(lat_ref) * np.cos(lon_ref)
    y_ref = (N + alt_ref) * np.cos(lat_ref) * np.sin(lon_ref)
    z_ref = (N * (1 - e2) + alt_ref) * np.sin(lat_ref)

    # Calculate ECEF coordinates of new point
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (N * (1 - e2) + alt) * np.sin(lat)

    # Calculate differences in ECEF coordinates
    dx = x - x_ref
    dy = y - y_ref
    dz = z - z_ref

    # Transform ECEF to ENU
    t = np.array([[-np.sin(lon_ref), np.cos(lon_ref), 0],
                  [-np.sin(lat_ref)*np.cos(lon_ref), -np.sin(lat_ref)*np.sin(lon_ref), np.cos(lat_ref)],
                  [np.cos(lat_ref)*np.cos(lon_ref), np.cos(lat_ref)*np.sin(lon_ref), np.sin(lat_ref)]])

    enu = np.dot(t, np.array([dx, dy, dz]))
    
    return enu


wgs84 = CRS.from_epsg(4326)  # GPS coordinates (WGS84)
utm_zone33 = CRS.from_proj4("+proj=utm +zone=55 +datum=WGS84 +units=m +no_defs")  # UTM projection
transformer = Transformer.from_crs(wgs84, utm_zone33)
def callback(data: KonumBilgileri):
    # Parsing the incoming message
    marker_array = MarkerArray()
    for iha in data.konumBilgileri:
        if iha.takim_numarasi == 31:
            continue
        marker = create_marker(iha)
        marker_array.markers.append(marker)

    # Assign unique IDs to each marker in the array
    for i, marker in enumerate(marker_array.markers):
        marker.id = i

    marker_pub.publish(marker_array)


def create_marker(iha: KonumBilgisi):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "iha_visualization"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    x, y, z = geodetic_to_enu(iha.IHA_enlem, iha.IHA_boylam, iha.IHA_irtifa)
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z

    roll = math.radians(iha.IHA_yatis)
    pitch = math.radians(iha.IHA_dikilme)
    yaw = math.radians(iha.IHA_yonelme)
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    marker.pose.orientation = Quaternion(*quaternion)

    marker.scale.x = 25.0
    marker.scale.y = 25.0
    marker.scale.z = 25.0

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    return marker

def listener():
    rospy.init_node('iha_subscriber', anonymous=True)
    rospy.Subscriber("/konum_bilgileri", KonumBilgileri, callback)
    rospy.spin()

if __name__ == '__main__':
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass