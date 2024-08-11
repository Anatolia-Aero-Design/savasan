#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from mavros_msgs.msg import AttitudeTarget
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from autonom_maneuver import AutonomNode
import utilities as utils
from pyzbar.pyzbar import decode


class QR_Node:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        
        # Initialize image subscribers as None
        self.image_sub = None 
        self.waypoint_sub = None
        
        self.qr_data = None

        
        self.qr_pub = rospy.Publisher('/qr_code_data', String, queue_size=10)
        self.image_pub = rospy.Publisher('camera/kamikaze_image', Image, queue_size=60)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        
        # Services to start and stop the mission
        self.start_service = rospy.Service('start_kamikaze', Empty, self.start_mission) 
    
    def start_mission(self, req):
        if self.image_sub is None:
            self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
            rospy.loginfo("Kamikaze mission started.")
        return EmptyResponse()
    
    def abort_mission(self, req):
        ...
    
    def process_image(self, image):
        image = utils.adjust_brightness_contrast(image, brightness=30, contrast=20)
        image = utils.gamma_correction(image, gamma=1.2)
        image = utils.noise_reduction(image)
        image = utils.adaptive_threshold(image)
        return image
    
    def qr_reader(self, frame):
        image = self.process_image(frame)
        decoded_objects = decode(image)
        for obj in decoded_objects:
            self.qr_data = obj.data.decode("utf-8")
            self.qr_pub.publish(self.qr_data)
            rospy.loginfo(f"QR Code Data: {self.qr_data}")
        return image
    
    def callback(self, image_msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        image = self.qr_reader(frame)
        try:
            # Convert OpenCV image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(image, "mono8")
            processed_image_msg.header = image_msg.header
            self.image_pub.publish(processed_image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
    
if __name__ == '__main__':
    rospy.init_node('QR_node', anonymous=True)
    qr_node = QR_Node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down QR node")