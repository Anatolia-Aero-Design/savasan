#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from std_srvs.srv import Empty, EmptyResponse
from image_processor.msg import Yolo_xywh  # Import the custom message

class YOLOv8TrackingNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = None  # Subscriber will be created in start_tracking
        self.bbox_pub = rospy.Publisher("/yolov8/xywh", Yolo_xywh, queue_size=100)

        # Load YOLOv8 model
        self.model = YOLO("/home/auki/catkin_ws/src/savasan/yolov8/model/best_x.pt")

        # Services to start and stop tracking
        self.start_service = rospy.Service('start_yolov8_tracking', Empty, self.start_tracking)
        self.stop_service = rospy.Service('stop_yolov8_tracking', Empty, self.stop_tracking)

    def start_tracking(self, req):
        if self.image_sub is None:
            self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
            rospy.loginfo("YOLOv8 tracking started.")
        return EmptyResponse()

    def stop_tracking(self, req):
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
            rospy.loginfo("YOLOv8 tracking stopped.")
        return EmptyResponse()

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = self.model.track(frame, persist=True)

        #TODO: handle result and publish

if __name__ == '__main__':
    rospy.init_node('yolov8_tracking_node', anonymous=True)
    yolov8_tracking_node = YOLOv8TrackingNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLOv8 tracking node")
