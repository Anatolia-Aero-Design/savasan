#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from mavros_msgs.msg import AttitudeTarget
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import utilities as utils
from pyzbar.pyzbar import decode

class QR_Node:
    def __init__(self) -> None:
        self.bridge = CvBridge()    
        self.read = False 
        self.qr_data = None
        
        # Initialize image subscriber
        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        # Initialize publishers 
        self.qr_pub = rospy.Publisher('/qr_code_data', String, queue_size=10)
        self.image_pub = rospy.Publisher('camera/kamikaze_image', Image, queue_size=60)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        # Get server API as parameter
        self.server_url_kamikaze_bilgisi = rospy.get_param('/comm_node/api/kamikaze_bilgisi')
    
    def read_check(self):
        return self.read
    
    def process_image(self, image):
        image = utils.adjust_brightness_contrast(image, brightness=30, contrast=20)
        image = utils.gamma_correction(image, gamma=1.2)
        image = utils.noise_reduction(image)
        image = utils.adaptive_threshold(image)
        return image
    
    def qr_reader(self, frame):
        image = self.process_image(frame)
        decoded_objects = decode(image)
        if decoded_objects:
            self.read = True
            for obj in decoded_objects:
                self.qr_data = obj.data.decode("utf-8")
                self.qr_pub.publish(self.qr_data)
                rospy.loginfo(f"QR Code Data: {self.qr_data}")
        else:
            self.read = False
        return image
    
    def image_callback(self, image_msg):
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