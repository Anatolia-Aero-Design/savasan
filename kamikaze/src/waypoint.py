#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear,CommandLong
import math
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
import utilities as utils
from geometry_msgs.msg import PoseStamped
from qr_reader import QRReader
import numpy as np




class WaypointNode:
        
    def __init__(self):
        
        self.uav_position = None
            
        self.HOME_LATITUDE = 39.819850
        self.HOME_LONGITUDE = 30.534169
        self.HOME_ALTITUDE = 100
        
        self.ROAD_DISTANCE = 300

        self.TARGET_LATITUDE = 39.82006598
        self.TARGET_LONGITUDE = 30.53404039
        self.TARGET_ALTITUDE = 10.0
        
        """self.azimuth_degree = rospy.get_param("loremippsum")
        self.dive_angle = rospy.get_param("loreippsum")
        self.distance_of_approach = rospy.get_param("loreippsum")
        self.offset = rospy.get_param("loreippsum")"""
        
        
        clear_waypoints_service = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        rospy.wait_for_service('/mavros/mission/clear')
        clear_response = clear_waypoints_service()
        rospy.loginfo("Waypoints cleared: %s", clear_response.success)
            
        self.start_service = rospy.Service("start_kamikaze", Empty, self.start_waypoint)
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.uav_pose_callback
        )
        self.rel_altitude_sub = rospy.Subscriber(
            "/mavros/global_position/rel_alt", Float64, self.rel_altitude_callback
        )
        

    def rel_altitude_callback(self, data):
        self.altitude = float(data.data)  # Extract data from the message
        
    def create_circle_around_origin(self,radius):
        
        origin_enu = utils.geodetic2enu(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE,
                              self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
        enu_coords = []
        geodetic_coords = []
        
    #waypoiont altitudeları uçağın servis çağırıldığı andaki altitudeuna göre setlenmektedir !!!
        
        azimuth_angle = self.azimuth_angle
        #özel dereceler için negatif dönüşümler verileceK
        
        if self.offset != 0 :
            origin_enu = utils.geodetic2enu(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE,
                              self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
            x, y = utils.calculate_coordinates(self.offset, azimuth_angle)
            origin_enu = (x, y, origin_enu[2])
            lat, lon, alt = utils.enu_to_geodetic(origin_enu[0], origin_enu[1], origin_enu[2], self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
            self.TARGET_LATITUDE = lat 
            self.TARGET_LONGITUDE = lon
            self.TARGET_ALTITUDE= alt 
            
        for radius in self.radii:
            x, y = utils.calculate_coordinates(radius, azimuth_angle)
            enu_coords.append((x, y, self.altitude - self.TARGET_ALTITUDE))
            
        x, y = utils.exit_calculate_coordinates(self.exitradius, azimuth_angle)
        enu_coords.append((x, y, self.altitude - self.TARGET_ALTITUDE))
            

            
        for east, north, up in enu_coords:
            lat, lon, alt = utils.enu_to_geodetic(east, north, up, self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE)
            geodetic_coords.append((lat, lon, alt))
            print(f"ENU: (E: {east}, N: {north}, U: {up}) -> Lat: {lat}, Lon: {lon}, Alt: {alt}")
            
        return geodetic_coords
    
    def start_waypoint(self,waypoints):
        self.qr_reader = QRReader()
        
        self.distance_of_approach = 150 # örnek
        self.dive_angle = 40 # örnek
        self.dive_angle_rad = math.radians(self.dive_angle)
        
        self.small = (self.altitude - self.TARGET_ALTITUDE)/math.tan(self.dive_angle_rad )
        big = self.distance_of_approach + self.small
        # big 1.waypointin hedefe uzaklığı small 2.wawypointin hedefe uzaklığı
        
 
        self.radii = [big , self.small] 
        self.exitradius = 200.0     # target pointle çıkış pointi arası mesafe istersen setleyebilirsin
        self.azimuth_angle =270     # compassa göre dalınacak yön seçimi
        self.offset = 0             # compassa göre seçilen açıda tüm hedeflerin kaç metre kaydırılacağı
        self.offset_circle = big + self.offset
        
        #bu değerleri init içerisinde vermeye çalışırsan altitude verisi daha gelmediği için hata veriyor
        
        
        radius = self.radii[0] 
        geodetic_coords = self.create_circle_around_origin(radius)  # Get geodetic coordinates

        if not geodetic_coords or len(geodetic_coords) < 3:
            rospy.logerr("Insufficient geodetic coordinates generated.")
            return EmptyResponse()
        
        waypoints = self.waypoints(geodetic_coords) 

        # Modify waypoints_with_angle() function to accept an angle_degrees parameter
    def waypoints(self, geodetic_coords):
        # Define waypoints
        waypoints = WaypointList()

        # Home Waypoint (WP0)
        wp0 = Waypoint()
        wp0.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp0.command = 16
        wp0.is_current = False  
        wp0.autocontinue = True
        wp0.param1 = 0
        wp0.param2 = 0
        wp0.param3 = 0
        wp0.param4 = 0.0
        wp0.x_lat = self.HOME_LATITUDE
        wp0.y_long = self.HOME_LONGITUDE
        wp0.z_alt = self.HOME_ALTITUDE
        waypoints.waypoints.append(wp0)

        # Waypoint 1: First calculated waypoint
        wp1 = Waypoint()
        wp1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp1.command = 16
        wp1.is_current = True  
        wp1.autocontinue = True
        wp1.param1 = 0
        wp1.param2 = 0
        wp1.param3 = 0
        wp1.param4 = 0.0
        wp1.x_lat = geodetic_coords[0][0]  # First latitude from geodetic_coords
        wp1.y_long = geodetic_coords[0][1]  # First longitude from geodetic_coords
        wp1.z_alt = geodetic_coords[0][2]
        waypoints.waypoints.append(wp1)

        # Waypoint 2: Second calculated waypoint
        wp2 = Waypoint()
        wp2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp2.command = 16
        wp2.is_current = False  
        wp2.autocontinue = True
        wp2.param1 = 0
        wp2.param2 = 0
        wp2.param3 = 0
        wp2.param4 = 0.0
        wp2.x_lat = geodetic_coords[1][0]  # Second latitude from geodetic_coords
        wp2.y_long = geodetic_coords[1][1]  # Second longitude from geodetic_coords
        wp2.z_alt = geodetic_coords[1][2]
        waypoints.waypoints.append(wp2)

        # Waypoint 3: Road waypoint (fixed)
        wp3 = Waypoint()
        wp3.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp3.command = 16
        wp3.is_current = False  
        wp3.autocontinue = True
        wp3.param1 = 0
        wp3.param2 = 0
        wp3.param3 = 0
        wp3.param4 = 0.0
        wp3.x_lat = self.TARGET_LATITUDE
        wp3.y_long = self.TARGET_LONGITUDE
        wp3.z_alt = self.TARGET_ALTITUDE
        waypoints.waypoints.append(wp3)

        # Waypoint 4: Third calculated waypoint
        wp4 = Waypoint()
        wp4.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp4.command = 16
        wp4.is_current = False  
        wp4.autocontinue = True
        wp4.param1 = 0
        wp4.param2 = 0
        wp4.param3 = 0
        wp4.param4 = 0.0
        wp4.x_lat = geodetic_coords[2][0]  # Third latitude from geodetic_coords
        wp4.y_long = geodetic_coords[2][1]  # Third longitude from geodetic_coords
        wp4.z_alt = geodetic_coords[2][2]
        waypoints.waypoints.append(wp4)

        return waypoints
            
         
if __name__ == '__main__':
    rospy.init_node("waypoint_node", anonymous=True)
    reposition = WaypointNode()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed during node initialization {e}")
    