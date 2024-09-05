#!/usr/bin/env python

import rospy
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix

def home_position_callback(msg):
    navsatfix_msg = NavSatFix()

    navsatfix_msg.header.stamp = rospy.Time.now()
    navsatfix_msg.header.frame_id = "map"

    navsatfix_msg.latitude = msg.geo.latitude
    navsatfix_msg.longitude = msg.geo.longitude
    navsatfix_msg.altitude = msg.geo.altitude

    # Publish the NavSatFix message
    navsatfix_pub.publish(navsatfix_msg)

if __name__ == '__main__':
    rospy.init_node('home_position_publisher', anonymous=True)

    # Create a publisher for the NavSatFix message
    navsatfix_pub = rospy.Publisher('/home_position/navsatfix', NavSatFix, queue_size=10)

    # Create a subscriber for the HomePosition message
    rospy.Subscriber('/mavros/home_position/home', HomePosition, home_position_callback)

    # Keep the node running
    rospy.spin()