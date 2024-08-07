#!/usr/bin/env python

import rospy
from server_comm.msg import KonumBilgileri, KonumBilgisi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
import tf.transformations
from pyproj import CRS, Transformer
import math
import numpy as np


# WGS84 ellipsoid constants
a = 6378137.0         # Semi-major axis
f = 1 / 298.257223563 # Flattening
e2 = 2*f - f**2  

def geodetic_to_enu(lat, lon, alt):
    lat_ref = -35.3632622
    lon_ref = 149.1652375
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

previous_positions = []
received_waypoints = []

def waypoint_callback(msg):
    # Extract waypoint coordinates
    waypoints = msg.data
    rospy.loginfo(f"Received waypoints: {waypoints}")

def callback(data: KonumBilgileri):
    global previous_positions

    # Parsing the incoming message
    marker_array = MarkerArray()
    trajectory_marker = create_trajectory_marker()
    for iha in data.konumBilgileri:
        if iha.takim_numarasi == 31:
            continue
        marker = create_marker(iha)
        marker_array.markers.append(marker)

        # Add the current position to the trajectory marker
        current_position = Point(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
        previous_positions.append(current_position)
        trajectory_marker.points.append(current_position)

    # Assign unique IDs to each marker in the array
    for i, marker in enumerate(marker_array.markers):
        marker.id = i

    marker_array.markers.append(trajectory_marker)
    marker_pub.publish(marker_array)
    publish_waypoints()


def create_marker(x, y, z, ns, marker_id, color):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.scale.x = 10.0
    marker.scale.y = 10.0
    marker.scale.z = 10.0
    marker.color.a = 1.0
    marker.color.r, marker.color.g, marker.color.b = color
    
    return marker

def create_trajectory_marker():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "trajectory"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 5.0  # Line width
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    return marker

def publish_waypoints():
    waypoints = [
        (-35.3632622, 149.1652375, 0),  # Waypoint 1
        (-35.36538764, 149.160, 0),     # Waypoint 2
        (-35.36540000, 149.161, 0),     # Waypoint 3
        (-35.36540000, 149.162, 0),     # Waypoint 4
        (-35.36540000, 149.163, 0)      # Waypoint 5
    ]

    marker_array = MarkerArray()
    for i, (lat, lon, alt) in enumerate(waypoints):
        x, y, z = geodetic_to_enu(lat, lon, alt)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker_array.markers.append(marker)
    
    waypoint_pub.publish(marker_array)


def listener():
    rospy.init_node('iha_subscriber', anonymous=True)
    rospy.Subscriber("/konum_bilgileri", KonumBilgileri, callback)
    rospy.Subscriber("/waypoint_coordinates", Float32MultiArray, waypoint_callback)
    rospy.spin()

if __name__ == '__main__':
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    waypoint_pub = rospy.Publisher('visualization_waypoints', MarkerArray, queue_size=10)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
