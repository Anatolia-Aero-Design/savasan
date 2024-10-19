#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from mavros_msgs.msg import AttitudeTarget
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import utilities as utils
from pyzbar.pyzbar import decode


class QR_Node:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.read = False
        self.qr_data = ""
        self.image_sub = None

        self.image_pub = rospy.Publisher(
            'camera/kamikaze_image', Image, queue_size=60)

        # Services to start and stop tracking
        self.start_service = rospy.Service(
            'start_Qr', Trigger, self.start_qr)
        self.stop_service = rospy.Service(
            'stop_Qr', Trigger, self.stop_qr)

    def start_qr(self, req):
        try:
            if self.image_sub is None:
                self.image_sub = rospy.Subscriber(
                    'camera/image_raw', Image, self.image_callback)
                rospy.loginfo("Qr Reader started.")
            return TriggerResponse(success=1)
        except:
            return TriggerResponse(success=0)

    def stop_qr(self, req):
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None
                rospy.loginfo("Qr Reader stopped.")
            return TriggerResponse(success=1, message=self.qr_data)
        except:
            return TriggerResponse(success=0)

    def read_check(self):
        return self.read

    def process_image(self, image):
        image = utils.convert_to_grayscale(image)
        image = utils.edge_detection(image)
        image = utils.adjust_brightness_contrast(
            image, brightness=30, contrast=20)
        image = utils.gamma_correction(image, gamma=1.2)
        image = utils.noise_reduction(image)
        return image

    def qr_reader(self, frame):
        image = self.process_image(frame) 
        decoded_objects = decode(frame)

        if decoded_objects:
            self.read = True
            for obj in decoded_objects:
                self.qr_data = obj.data.decode("utf-8")
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
