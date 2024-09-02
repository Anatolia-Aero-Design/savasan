#!/usr/bin/env python

from std_srvs.srv import Empty
import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import WaypointPush, WaypointClear
import numpy as np
import utilities as utils

class Waypoint_node:
    def __init__(self) -> None:
        self.uav_position = None
        
        self.HOME_LATITUDE = 39.819850
        self.HOME_LONGITUDE = 30.534169
        self.HOME_ALTITUDE = 100
        
        self.TARGET_LATITUDE = 39.82006598
        self.TARGET_LONGITUDE = 30.53404039
        self.TARGET_ALTITUDE = 20
        
        # Get parameters from GUI // param name will be inserted here from GUI
        '''try:
            self.azimuth_degree = rospy.get_param('Lorem_Ipsum') 
            self.dive_angle = rospy.get_param('Lorem_Ipsum') 
            self.distance_of_approach = rospy.get_param('Lorem_Ipsum') 
            self.offset = rospy.get_param('Lorem_Ipsum') 
        except KeyError as e:
            rospy.logerr(f"Parameter not found!: {e}")'''
        
        self.uav_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.uav_pose_callback)
        
        self.start_service = rospy.Service('start_waypoint', Empty, self.waypoints)
        self.server_url_kamikaze_bilgisi = rospy.get_param('/comm_node/api/kamikaze_bilgisi')

    def uav_pose_callback(self, data):
        self.uav_position = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
          
    def push_waypoints(self, waypoints):
        rospy.wait_for_service('mission_push')
        clear_waypoints_service = rospy.ServiceProxy('mission_clear', WaypointClear)
        push_waypoints_service = rospy.ServiceProxy('mission_push', WaypointPush)
        try:
            response = clear_waypoints_service()
            response = push_waypoints_service(0, waypoints.waypoints)  
            return response.wp_transfered
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False        
    
    def waypoints(self, req):
        waypoints = WaypointList()

        wp1 = Waypoint()
        wp1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp1.command = 16
        wp1.is_current = True  
        wp1.autocontinue = True
        wp1.param1 = 0
        wp1.param2 = 0
        wp1.param3 = 0
        wp1.x_lat = self.HOME_LATITUDE
        wp1.y_long = self.HOME_LONGITUDE
        wp1.z_alt = self.HOME_ALTITUDE
        waypoints.waypoints.append(wp1)

        calculated_waypoints = utils.calculate_waypoint_sequence(self.uav_position[0],self.uav_position[1], self.uav_position[2],
                                                                 self.TARGET_LATITUDE, self.TARGET_LONGITUDE, self.TARGET_ALTITUDE, 
                                                                 self.azimuth_degree, self.distance_of_approach, self.dive_angle, 100)
       # first two wp calculations are wrong
        
        wp2 = Waypoint()
        wp2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp2.command = 16
        wp2.is_current = False  
        wp2.autocontinue = True
        wp2.param1 = 0
        wp2.param2 = 0
        wp2.param3 = 0
        wp2.x_lat = calculated_waypoints[0][0]
        wp2.y_long = calculated_waypoints[0][1]
        wp2.z_alt = calculated_waypoints[0][2]
        waypoints.waypoints.append(wp2)

        wp3 = Waypoint()
        wp3.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp3.command = 16
        wp3.is_current = False  
        wp3.autocontinue = True
        wp3.param1 = 0
        wp3.param2 = 0
        wp3.param3 = 0
        wp3.x_lat = calculated_waypoints[1][0]
        wp3.y_long = calculated_waypoints[1][1]
        wp3.z_alt = calculated_waypoints[1][2]
        waypoints.waypoints.append(wp3)

        wp4 = Waypoint()
        wp4.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp4.command = 16
        wp4.is_current = False  
        wp4.autocontinue = True
        wp4.param1 = 0
        wp4.param2 = 0
        wp4.param3 = 0
        wp4.x_lat  = self.TARGET_LATITUDE
        wp4.y_long = self.TARGET_LONGITUDE
        wp4.z_alt  = self.TARGET_ALTITUDE
        waypoints.waypoints.append(wp4)

        wp5 = Waypoint()
        wp5.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp5.command = 16
        wp5.is_current = False 
        wp5.autocontinue = True
        wp5.param1 = 0
        wp5.param2 = 0
        wp5.param3 = 0
        wp5.x_lat  = calculated_waypoints[3][0]
        wp5.y_long = calculated_waypoints[3][1]
        wp5.z_alt  = calculated_waypoints[3][2]
        waypoints.waypoints.append(wp5)

        success = self.push_waypoints(waypoints)
        rospy.logerr(waypoints)
        
        if success:
            rospy.loginfo("Waypoints pushed successfully!")
        else:
            rospy.logerr("Failed to push waypoints!")

if __name__ == '__main__':
    rospy.init_node('waypoint_node', anonymous=True)
    waypoints = Waypoint_node()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed during node initialization {e}")