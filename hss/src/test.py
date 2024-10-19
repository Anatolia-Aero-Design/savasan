#!/usr/bin/env python

import rospy
from mavros_msgs.srv import WaypointPush, WaypointPushRequest
from mavros_msgs.msg import Waypoint


def draw_empty_fences():
    coordinates = [[36.942314, 35.563323], [36.942673, 35.553363], [
        36.937683, 35.553324], [36.937864, 35.562873]]
    fence_points = WaypointPushRequest()
    for index, coordinate in enumerate(coordinates):
        fence_point = Waypoint()
        fence_point.frame = 3  # GLOBAL (relative altitude)
        fence_point.command = 5001  # NAV_FENCE_POINT
        fence_point.is_current = False  # Not a current waypoint
        fence_point.autocontinue = True  # Autocontinue
        fence_point.param1 = 4
        fence_point.param2 = index
        fence_point.x_lat = coordinate[0]  # Latitude
        fence_point.y_long = coordinate[1]  # Longitude
        fence_point.z_alt = 100
        fence_points.waypoints.append(fence_point)
    return fence_points


def push_fence():
    rospy.init_node('push_fence_node', anonymous=True)

    # Wait for the MAVROS service to push waypoints (fence points)
    rospy.wait_for_service('/mavros/geofence/push')

    try:
        # Define the service for pushing waypoints
        push_wp = rospy.ServiceProxy('/mavros/geofence/push', WaypointPush)

        # Create a simple fence point (as a waypoint)
        fence_point = Waypoint()
        fence_point.frame = 3  # GLOBAL (relative altitude)
        fence_point.command = 5004  # NAV_FENCE_POINT
        fence_point.is_current = False  # Not a current waypoint
        fence_point.autocontinue = True  # Autocontinue
        fence_point.param1 = 30  # For circular fences
        fence_point.x_lat = 36.9376322  # Latitude
        fence_point.y_long = 35.5632588  # Longitude
        fence_point.z_alt = 100  # Altitude in meters

        # Create a WaypointPushRequest and append the fence point
        wp_push_request = draw_empty_fences()
        wp_push_request.waypoints.append(fence_point)

        # Call the service and push the fence point
        response = push_wp(wp_push_request)
        if response.success:
            rospy.loginfo("Fence point pushed successfully!")
        else:
            rospy.logwarn("Failed to push fence point.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == '__main__':
    try:
        push_fence()
    except rospy.ROSInterruptException:
        pass
