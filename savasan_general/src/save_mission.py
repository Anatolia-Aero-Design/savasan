#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList
import json
from mavros_msgs.srv import WaypointPull


def waypoint_callback(msg):
    waypoints = msg.waypoints

    mission_data = []

    # Convert the waypoints to a JSON-serializable format
    for wp in waypoints:
        wp_data = {
            'frame': wp.frame,
            'command': wp.command,
            'is_current': wp.is_current,
            'autocontinue': wp.autocontinue,
            'param1': wp.param1,
            'param2': wp.param2,
            'param3': wp.param3,
            'param4': wp.param4,
            'x_lat': wp.x_lat,
            'y_long': wp.y_long,
            'z_alt': wp.z_alt,
        }
        mission_data.append(wp_data)

    # Save to file
    save_mission_to_file(mission_data)


def save_mission_to_file(mission_data):
    filename = 'missions.json'

    with open(filename, 'w') as f:
        json.dump(mission_data, f, indent=4)

    rospy.loginfo("Mission saved to file: {}".format(filename))


def mission_downloader():

    # Subscribe to the /mavros/mission/waypoints topic
    rospy.Subscriber('/mavros/mission/waypoints',
                     WaypointList, waypoint_callback)

    # Spin to keep the script alive and continuously process incoming messages
    rospy.spin()


def download_mission():
    # Initialize ROS node
    rospy.init_node('mission_downloader', anonymous=True)

    # Wait for the WaypointPull service to become available
    rospy.wait_for_service('/mavros/mission/pull')

    try:
        # Create a service proxy for pulling the mission
        pull_service = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)

        # Call the service to pull the mission
        response = pull_service()

        if response.success:
            rospy.loginfo("Successfully pulled mission with {} waypoints".format(
                response.wp_received))
            return None
        else:
            rospy.logerr("Failed to pull mission")
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return None


if __name__ == '__main__':
    try:
        download_mission()
        mission_downloader()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
