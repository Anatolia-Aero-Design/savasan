#!/usr/bin/env python

import rospy
import tf
import geodesy.utm
from geometry_msgs.msg import PointStamped

def gps_to_map(lat, lon, alt):
    rospy.init_node('gps_to_map_transformer')
    
    listener = tf.TransformListener()
    
    utm_point = geodesy.utm.fromLatLong(lat, lon).toPoint()
    new_x = utm_point.x
    new_y = utm_point.y
    new_z = alt
    
    gps_point = PointStamped()
    gps_point.header.frame_id = "gps_origin"
    gps_point.header.stamp = rospy.Time.now()
    gps_point.point.x = new_x
    gps_point.point.y = new_y
    gps_point.point.z = new_z
    
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            transformed_point = listener.transformPoint("map", gps_point)
            rospy.loginfo("Transformed Point: x=%f, y=%f, z=%f",
                          transformed_point.point.x,
                          transformed_point.point.y,
                          transformed_point.point.z)
            return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    try:
        new_lat = 47.398742
        new_lon = 8.546594
        new_alt = 10.0
        
        x, y, z = gps_to_map(new_lat, new_lon, new_alt)
        print(f"Map Frame Coordinates: x={x}, y={y}, z={z}")
    except rospy.ROSInterruptException:
        pass
