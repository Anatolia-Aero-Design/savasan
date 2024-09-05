#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear, CommandLong,WaypointClear
import math
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
import utilities as utils
from geometry_msgs.msg import PoseStamped
from QR_node import QR_Node
import numpy as np


class UAVWaypointManager:
    def __init__(self):
        """Initialize the UAV waypoint manager, set parameters, and start ROS services."""
        # UAV state
        self.uav_altitude = None
        self.uav_position = None

        # Home coordinates
        self.home_latitude = float(rospy.get_param("/comm_node/Home_Lat"))
        self.home_longitude = float(rospy.get_param("/comm_node/Home_Lon"))
        self.home_altitude = float(rospy.get_param("/comm_node/Home_Alt"))

        # Target coordinates
        self.target_latitude = float(rospy.get_param("/comm_node/Target_Lat"))
        self.target_longitude = float(rospy.get_param("/comm_node/Target_Lon"))
        self.target_altitude = float(rospy.get_param("/comm_node/Target_Alt"))
        

        # Flight parameters
        self.road_distance = 300



        # ROS services and subscribers
        self.start_service = rospy.Service("start_kamikaze", Trigger, self.handle_start_mission)
        self.pose_subscriber = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.update_uav_pose
        )
        self.altitude_subscriber = rospy.Subscriber(
            "/mavros/global_position/rel_alt", Float64, self.update_uav_altitude
        )

    def update_uav_altitude(self, altitude_data):
        """Update the UAV altitude from the subscriber callback."""
        self.uav_altitude = float(altitude_data.data)

    def update_uav_pose(self, pose_data):
        """Update the UAV position from the subscriber callback."""
        self.uav_position = pose_data.pose

    def calculate_radii(self, altitude, dive_angle):
        """
        Calculate the radii for the waypoint circle based on altitude and dive angle.
        
        :param altitude: The current altitude of the UAV.
        :param dive_angle: The dive angle in radians.
        :return: The larger and smaller radii of the circle.
        """
        small_radius = (altitude - self.target_altitude) / math.tan(dive_angle)
        large_radius = self.approach_distance + small_radius
        return large_radius, small_radius

    def calculate_waypoint_coordinates(self, radii, azimuth_angle, altitude):
        """
        Calculate geodetic coordinates for the waypoints around the target.

        :param radii: A tuple of radii (large_radius, small_radius) for the circle.
        :param azimuth_angle: The azimuth angle in degrees.
        :param altitude: Current altitude of the UAV.
        :return: A list of geodetic coordinates (latitude, longitude, altitude).
        """
        large_radius, small_radius = radii
        enu_coords = []

        target_x,target_y = utils.calculate_coordinates(0+self.circle_offset, azimuth_angle)
        # Calculate ENU coordinates for the circular path
        for radius in [large_radius, small_radius]:
            x, y = utils.calculate_coordinates(radius, azimuth_angle)
            enu_coords.append((-x+target_x, -y+target_y, altitude - self.target_altitude))
            
        enu_coords.append((target_x, target_y, self.target_altitude))

        # Calculate final exit point coordinates
        exit_x, exit_y = utils.calculate_coordinates(small_radius, azimuth_angle)
        enu_coords.append((exit_x+target_x, exit_y+target_y, self.target_altitude+20))
        
        exit_x2, exit_y2 = utils.calculate_coordinates(large_radius, azimuth_angle)
        enu_coords.append((exit_x2+target_x, exit_y2+target_y, altitude - self.target_altitude))

        # Convert ENU to geodetic coordinates
        geodetic_coords = []
        for east, north, up in enu_coords:
            lat, lon, alt = utils.enu_to_geodetic(east, north, up, self.target_latitude, self.target_longitude, self.target_altitude)
            geodetic_coords.append((lat, lon, alt))

        return geodetic_coords

    def create_waypoints(self, geodetic_coords):
        """
        Generate MAVROS WaypointList from geodetic coordinates.

        :param geodetic_coords: List of geodetic coordinates (latitude, longitude, altitude).
        :return: A populated WaypointList.
        """
        if not geodetic_coords or len(geodetic_coords) < 3:
            rospy.logerr("Insufficient geodetic coordinates generated.")
            return None

        waypoints = WaypointList()

        # Home Waypoint (WP0)
        wp_home = Waypoint()
        wp_home.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp_home.command = 16
        wp_home.is_current = True
        wp_home.autocontinue = True
        wp_home.x_lat = self.home_latitude
        wp_home.y_long = self.home_longitude
        wp_home.z_alt = 0
        waypoints.waypoints.append(wp_home)

        # Add calculated waypoints
        for idx, coord in enumerate(geodetic_coords):
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command = 16
            wp.is_current = False  # Only WP0 is the starting point
            wp.autocontinue = True
            wp.x_lat = coord[0]
            wp.y_long = coord[1]
            wp.z_alt = coord[2]
            waypoints.waypoints.append(wp)
        print(waypoints)
        return waypoints

    def handle_start_mission(self, req):
        """Handle the start mission service call."""
        if self.uav_altitude is None:
            rospy.logerr("UAV altitude not available.")
            return TriggerResponse(success=False)
        
        self.azimuth_angle = rospy.get_param("azimuth_angle", 0.0)  
        self.dive_angle = math.radians(rospy.get_param("dive_angle", 30) )
        self.approach_distance = rospy.get_param("approach_distance", 300)
        self.circle_offset = rospy.get_param("circle_offset", 0)
        
        # Calculate the radii for waypoints
        radii = self.calculate_radii(self.uav_altitude, self.dive_angle)

        # Generate geodetic coordinates for the waypoint circle
        geodetic_coords = self.calculate_waypoint_coordinates(radii, self.azimuth_angle, self.uav_altitude)

        # Generate waypoints from the coordinates
        waypoints = self.create_waypoints(geodetic_coords)
        if waypoints is None:
            return TriggerResponse(success=False)

        rospy.wait_for_service('/mavros/mission/clear')
        try:
            clear_wp = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            response = clear_wp()
            if response.success:
                rospy.loginfo("Waypoints successfully cleared.")
                
            else:
                rospy.logwarn("Failed to clear waypoints.")
        
        except rospy.ServiceException as e:
            rospy.logerr("Service Clear Waypoint call failed: %s", e)
            return TriggerResponse(success=False)
        
        rospy.wait_for_service('/mavros/mission/push')
        try:
            waypoint_push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            response = waypoint_push_service(0, waypoints.waypoints)
            rospy.loginfo("Waypoints successfully generated and pushed.")
            
        except rospy.ServiceException as e:
            rospy.logerr("Waypoints generated and pushed Failed: %s", e)
            return TriggerResponse(success=False)

        
        
        
        
        return TriggerResponse(success=True)
        


if __name__ == '__main__':
    rospy.init_node("uav_waypoint_manager", anonymous=True)
    waypoint_manager = UAVWaypointManager()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("UAV waypoint manager interrupted.")
