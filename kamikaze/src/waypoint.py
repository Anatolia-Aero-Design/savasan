#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear
import math
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
import utilities as utils
from geometry_msgs.msg import PoseStamped


class WaypointNode:
        
    def __init__(self):
        
        self.uav_position = None
            
        self.HOME_LATITUDE = 39.819850
        self.HOME_LONGITUDE = 30.534169
        self.HOME_ALTITUDE = 100
        
        self.ROAD_DISTANCE = 300

        self.TARGET_LATITUDE = 39.82006598
        self.TARGET_LONGITUDE = 30.53404039
        self.TARGET_ALTITUDE = 100.0
            
        self.start_service = rospy.Service("start_kamikaze", Empty, self.start_waypoint)
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.uav_pose_callback
        )
        self.rel_altitude_sub = rospy.Subscriber(
            "/mavros/global_position/rel_alt", Float64, self.rel_altitude_callback
        )
        
        self.server_url_kamikaze_bilgisi = rospy.get_param('/comm_node/api/kamikaze_bilgisi')
        
    def uav_pose_callback(self, data):
        self.uav_x, self.uav_y, self.uav_z = (
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
        )
        self.roll, self.pitch, self.yaw = utils.quaternion_to_euler(
            data.pose.orientation.w,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
        )
        
    def rel_altitude_callback(self, data):
        self.altitude = float(data.data)  # Extract data from the message
            
    
    def push_waypoints(self, waypoint_list):
        rospy.wait_for_service('/mavros/mission/push')
        rospy.wait_for_service('/mavros/mission/clear')

        try:
            clear_waypoints_service = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            push_waypoints_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            
            # Clear existing waypoints
            clear_response = clear_waypoints_service()
            rospy.loginfo("Waypoints cleared: %s", clear_response.success)
            
            # Convert WaypointList to a list of Waypoint messages
            waypoints = list(waypoint_list.waypoints)
            
            # Push new waypoints
            response = push_waypoints_service(0, waypoints)  # 0 for mission type
            rospy.loginfo("Push response: %s", response)
            
            return response.wp_transfered
        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False



        # Modify waypoints_with_angle() function to accept an angle_degrees parameter
    def waypoints(self):
            
            
            # Define waypoints
            waypoints = WaypointList()

            # Waypoint 1: home position
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

            # Calculate waypoints with the specified angle
            calculated_waypoints = utils.calculate_waypoints_with_angle(self.TARGET_LATITUDE, self.TARGET_LONGITUDE, 70,50,50,150)

            # Existing code...
            # Waypoint 2: Road waypoint 1
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
            wp2.z_alt = self.HOME_ALTITUDE
            waypoints.waypoints.append(wp2)

            # Waypoint 3: Road waypoint 2
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
            wp3.z_alt = self.HOME_ALTITUDE
            waypoints.waypoints.append(wp3)

            # Waypoint 4: Target coordinates and target altitude
            wp4 = Waypoint()
            wp4.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp4.command = 16
            wp4.is_current = False  
            wp4.autocontinue = True
            wp4.param1 = 0
            wp4.param2 = 0
            wp4.param3 = 0
            wp4.x_lat = self.TARGET_LATITUDE
            wp4.y_long = self.TARGET_LONGITUDE
            wp4.z_alt = self.TARGET_ALTITUDE
            waypoints.waypoints.append(wp4)

            # Waypoint 5: Leave the target coordinates
            wp5 = Waypoint()
            wp5.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp5.command = 16
            wp5.is_current = False 
            wp5.autocontinue = True
            wp5.param1 = 0
            wp5.param2 = 0
            wp5.param3 = 0
            wp5.x_lat = calculated_waypoints[3][0]
            wp5.y_long = calculated_waypoints[3][1]
            wp5.z_alt = self.HOME_ALTITUDE
            waypoints.waypoints.append(wp5)
            rospy.loginfo(f"Generated waypoints: {[{'lat': wp.x_lat, 'lon': wp.y_long, 'alt': wp.z_alt} for wp in waypoints.waypoints]}")
            return waypoints
            
    def start_waypoint(self,waypoints):
        
            waypoints = self.waypoints()
            if not waypoints:
                rospy.logerr("No waypoints to push.")
                return EmptyResponse()

            # Print the latitude, longitude, and altitude of each waypoint
            rospy.loginfo(f"Waypoints to push: {[{'lat': wp.x_lat, 'lon': wp.y_long, 'alt': wp.z_alt} for wp in waypoints.waypoints]}")
    

            # Push waypoints to the flight controller
            result = self.push_waypoints(waypoints)
            if result:
                rospy.loginfo("Waypoints pushed successfully!")
            else:
                rospy.logerr("Failed to push waypoints!")
            
            return EmptyResponse()
         
if __name__ == '__main__':
    rospy.init_node("waypoint_node", anonymous=True)
    reposition = WaypointNode()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed during node initialization {e}")
    