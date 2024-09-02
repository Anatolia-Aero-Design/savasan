#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class PoseToMarker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_to_marker', anonymous=True)

        # Subscribe to the Pose topic
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Publisher for the Marker
        self.marker_pub = rospy.Publisher('/plane_model', Marker, queue_size=10)

        # Marker initialization
        self.marker = Marker()
        self.marker.header.frame_id = "map"  # Set the frame id to match your coordinate frame
        self.marker.type = Marker.MESH_RESOURCE      # Set the marker type; ARROW is common for representing poses
        self.marker.action = Marker.ADD
        self.marker.mesh_resource = "package://uav_visualization/models/shadow_mokab.dae"


        # Set some default marker properties
        self.marker.scale.x = 10  # Arrow length
        self.marker.scale.y = 10  # Arrow width
        self.marker.scale.z = 10  # Arrow height
        self.marker.color.a = 1.0  # Alpha (transparency)
        self.marker.color.r = 101/255  # Red
        self.marker.color.g = 107/255  # Green
        self.marker.color.b = 131/255  # Blue

    def pose_callback(self, msg):
        # Update marker position and orientation
        
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position = msg.pose.position
        self.marker.pose.orientation = msg.pose.orientation

        self.marker_pub.publish(self.marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PoseToMarker()
        node.run()
    except rospy.ROSInterruptException:
        pass
