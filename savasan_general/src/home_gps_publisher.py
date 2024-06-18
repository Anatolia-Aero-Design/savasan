#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandHome, CommandHomeRequest

def set_home_position(gps_data):
    rospy.wait_for_service('/mavros/cmd/set_home')
    try:
        set_home_srv = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        home_request = CommandHomeRequest()
        home_request.current_gps = True
        home_request.latitude = gps_data.latitude
        home_request.longitude = gps_data.longitude
        home_request.altitude = gps_data.altitude

        response = set_home_srv(home_request)
        if response.success:
            rospy.loginfo("Home position set successfully.")
        else:
            rospy.logerr("Failed to set home position.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def get_current_gps():
    try:
        gps_data = rospy.wait_for_message('/mavros/global_position/global', NavSatFix, timeout=10)
        return gps_data
    except rospy.ROSException as e:
        rospy.logerr("Failed to get current GPS position: %s", str(e))
        return None

def publish_home_gps():
    rospy.init_node('home_gps_publisher', anonymous=True)
    home_gps_pub = rospy.Publisher('home_gps', NavSatFix, queue_size=10)

    gps_data = get_current_gps()
    if gps_data is None:
        rospy.logerr("Unable to set home GPS position. Exiting.")
        return

    set_home_position(gps_data)

    rospy.loginfo("Home GPS position acquired: Lat: %f, Lon: %f, Alt: %f",
                  gps_data.latitude, gps_data.longitude, gps_data.altitude)

    rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        home_gps_pub.publish(gps_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_home_gps()
    except rospy.ROSInterruptException:
        pass
