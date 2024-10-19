#!/usr/bin/env python

from rospkg import RosPack
import rospy
from server_comm.msg import KonumBilgileri, KonumBilgisi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mavros_msgs.msg import HomePosition
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations
from pyproj import CRS, Transformer
import math
import numpy as np

rgb_list_values = [(125, 125, 0),
                   (0, 255, 0),
                   (0, 0, 255),
                   (255, 255, 0),
                   (255, 0, 255),
                   (0, 255, 255),
                   (128, 0, 0),
                   (0, 128, 0),
                   (0, 0, 128),
                   (128, 128, 0),
                   (128, 0, 128),
                   (0, 128, 128),
                   (192, 192, 192),
                   (128, 128, 128),
                   (255, 165, 0),
                   (255, 192, 203),
                   (0, 100, 0),
                   (139, 69, 19),
                   (75, 0, 130),
                   (173, 255, 47),
                   (135, 206, 250),
                   (244, 164, 96),
                   (240, 230, 140),
                   (220, 20, 60),
                   (46, 139, 87),
                   (255, 105, 180),
                   (72, 61, 139),
                   (255, 222, 173),
                   (154, 205, 50),
                   (186, 85, 211),
                   (0, 191, 255),
                   (147, 112, 219),
                   (60, 179, 113)]


class visialization():
    def __init__(self) -> None:

        # WGS84 ellipsoid constants
        self.a = 6378137.0         # Semi-major axis
        f = 1 / 298.257223563  # Flattening
        self.e2 = 2*f - f**2
        rospy.Subscriber("/konum_bilgileri", KonumBilgileri, self.callback)
        rospy.Subscriber("/mavros/home_position/home",
                         HomePosition, self.home_callback)

    # TODO: the home point is wrong
    def geodetic_to_enu(self, lat, lon, alt):

        lat_ref = self.lat
        lon_ref = self.lon
        alt_ref = self.alt
        # Convert degrees to radians
        lat_ref = np.deg2rad(lat_ref)
        lon_ref = np.deg2rad(lon_ref)
        lat = np.deg2rad(lat)
        lon = np.deg2rad(lon)

        # Calculate prime vertical radius of curvature
        N = self.a / np.sqrt(1 - self.e2 * np.sin(lat_ref)**2)

        # Calculate ECEF coordinates of reference point
        x_ref = (N + alt_ref) * np.cos(lat_ref) * np.cos(lon_ref)
        y_ref = (N + alt_ref) * np.cos(lat_ref) * np.sin(lon_ref)
        z_ref = (N * (1 - self.e2) + alt_ref) * np.sin(lat_ref)

        # Calculate ECEF coordinates of new point
        N = self.a / np.sqrt(1 - self.e2 * np.sin(lat)**2)
        x = (N + alt) * np.cos(lat) * np.cos(lon)
        y = (N + alt) * np.cos(lat) * np.sin(lon)
        z = (N * (1 - self.e2) + alt) * np.sin(lat)

        # Calculate differences in ECEF coordinates
        dx = x - x_ref
        dy = y - y_ref
        dz = z - z_ref

        # Transform ECEF to ENU
        t = np.array([[-np.sin(lon_ref), np.cos(lon_ref), 0],
                      [-np.sin(lat_ref)*np.cos(lon_ref), -np.sin(lat_ref)
                     * np.sin(lon_ref), np.cos(lat_ref)],
                      [np.cos(lat_ref)*np.cos(lon_ref), np.cos(lat_ref)*np.sin(lon_ref), np.sin(lat_ref)]])

        enu = np.dot(t, np.array([dx, dy, dz]))

        return enu

    def home_callback(self, data):
        self.lat = data.geo.latitude
        self.lon = data.geo.longitude
        self.alt = 0

    def callback(self, data: KonumBilgileri):
        self.target_id = rospy.get_param('/gps_navigator/target_id', default=1)
        # Parsing the incoming message
        marker_array = MarkerArray()
        for iha in data.konumBilgileri:
            if iha.takim_numarasi == 32:
                continue
            if iha.takim_numarasi == self.target_id:
                color = (255, 0, 0)
            else:
                color = (0,0,0)
            try:
                marker = self.create_marker(
                    iha, iha.takim_numarasi, color=color)
                marker_array.markers.append(marker)
            except Exception as e:
                print(e)

        # Assign unique IDs to each marker in the array
        for i, marker in enumerate(marker_array.markers):
            marker.id = i

        marker_pub.publish(marker_array)

    def create_marker(self, iha, takim_numarasi, color):
        # Create the main UAV model marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "iha_visualization"
        marker.id = 0  # Unique ID for the marker
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://uav_visualization/models/fixed_wing_plane_v2.dae"
        marker.action = Marker.ADD

        # Set position and orientation
        x, y, z = self.geodetic_to_enu(
            iha.IHA_enlem, iha.IHA_boylam, iha.IHA_irtifa)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        roll = math.radians(iha.IHA_yatis)
        pitch = math.radians(iha.IHA_dikilme)
        yaw = math.radians(iha.IHA_yonelme)
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        marker.pose.orientation = Quaternion(*quaternion)

        marker.scale.x = 3.0
        marker.scale.y = 3.0
        marker.scale.z = 3.0

        marker.color.a = 1.0
        if color:
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
        else:
            marker.color.r = rgb_list_values[takim_numarasi][0]
            marker.color.g = rgb_list_values[takim_numarasi][1]
            marker.color.b = rgb_list_values[takim_numarasi][2]
        return marker


if __name__ == '__main__':
    marker_pub = rospy.Publisher(
        'visualization_marker_array', MarkerArray, queue_size=10)
    try:
        rospy.init_node('uav_visualization', anonymous=True)
        helo = visialization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
