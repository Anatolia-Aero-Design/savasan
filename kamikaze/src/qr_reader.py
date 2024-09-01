#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyzbar.pyzbar as pyzbar
import numpy as np
from std_msgs.msg import String
import os
from datetime import datetime

class QRReader:
    def __init__(self):
        
        self.read = False
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        self.qr_pub = rospy.Publisher('/qr_code_data', String, queue_size=10)
        self.image_pub = rospy.Publisher('camera/kamikaze_image', Image, queue_size=60)
        
        self.server_url_kamikaze_bilgisi = rospy.get_param('/comm_node/api/kamikaze_bilgisi')
        
        self.save_path = '/home/poyrazzo/catkin_ws/src/savasan/kamikaze/qr_image'
        
    def read_check(self):
        return self.read

    def save_image(self, cv_image):
        # Create a filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.save_path, f"qr_image_{timestamp}.png")
        # Save the image
        cv2.imwrite(filename, cv_image)
        rospy.loginfo(f"Image saved as {filename}")
        
    def read_qr_code(self, data):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Process the image to detect QR codes
        decoded_objects = pyzbar.decode(cv_image)
        if decoded_objects:
            self.read = True
            rospy.loginfo("QR code detected!")
            for obj in decoded_objects:
                self.qr_data = obj.data.decode('utf-8')
                #rospy.loginfo(f"QR Code Data: {self.qr_data}")
                self.qr_pub.publish(self.qr_data)
                
        else:
            self.read = False
            
        

        # Optional: Display the image with QR codes detected
        for obj in decoded_objects:
            points = obj.polygon
            if len(points) > 4: 
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points

            n = len(hull)
            for j in range(0, n):
                cv2.line(cv_image, hull[j], hull[(j + 1) % n], (0, 255, 0), 3)

        # Display the frame (optional)
        cv2.imshow("QR Code Reader", cv_image)
        cv2.waitKey(1)


    def image_callback(self, data):
        self.read_qr_code(data)
        
        # Publish the image (for any additional processing)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.bridge.imgmsg_to_cv2(data, "bgr8"), "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish image: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        qr_reader = QRReader()
        qr_reader.run()
    except rospy.ROSInterruptException:
        pass
