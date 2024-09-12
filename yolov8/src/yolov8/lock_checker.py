#!/usr/bin/env python

from datetime import datetime

import requests
import rospy
from image_processor.msg import Yolo_xywh  # Import the custom message#
from std_msgs.msg import Bool
import time
from server_comm.srv import sendlock, sendlockRequest
from server_comm.msg import Kilitlenme


class Timer:
    def __init__(self):
        self.start_time = None
        self.end_time = None
        self.running = False

    def start(self):
        if not self.running:
            self.start_time = time.time()
            self.running = True
        else:
            pass

    def stop(self):
        if self.running:
            self.end_time = time.time()
            self.running = False
            elapsed_time = self.end_time - self.start_time
            return elapsed_time
        else:
            return None

    def reset(self):
        self.start_time = None
        self.end_time = None
        self.running = False

    def elapsed(self):
        if self.running:
            elapsed_time = time.time() - self.start_time
            return elapsed_time
        else:
            return None

    @staticmethod
    def format_time(seconds):
        mins, secs = divmod(seconds, 60)
        hours, mins = divmod(mins, 60)
        return f"{int(mins):02}:{secs:05.2f}"

    def get_current_time(self):
        now = datetime.now()
        return now.hour, now.minute, now.second, now.microsecond


class Lock_Checker:
    def __init__(self) -> None:
        self.session = requests.Session()
        self.lock_on_controller = None
        self.kilit_controller = None
        self.bbox = self.bbox_x = self.bbox_y = self.bbox_w = self.bbox_h = None

        self.xywh_sub = rospy.Subscriber(
            "/yolov8/xywh", Yolo_xywh, self.bbox_callback)

        self.kilit_pub = rospy.Publisher("/kilit", Bool, queue_size=60)
        self.timer = Timer()
        self.elapsed_time = None
        self.timer_check = False
        self.kilitlenme_data_sent = False

    def bbox_callback(self, msg):
        self.bbox = msg
        self.bbox_x = msg.x
        self.bbox_y = msg.y
        self.bbox_w = msg.w
        self.bbox_h = msg.h

        # get resolution parameters
        self.width = rospy.get_param("/camera_publisher/screen_width")
        self.height = rospy.get_param("/camera_publisher/screen_height")

        # Draw target area
        target_box_x = int(self.width * 0.25)
        target_box_y = int(self.height * 0.1)
        target_box_w = int(self.width * 0.75)
        target_box_h = int(self.height * 0.9)
        self.target_box_coordinates = (
            target_box_x,
            target_box_y,
            target_box_w,
            target_box_h,
        )
        bbox_coordinates = self.bbox_x, self.bbox_y, self.bbox_w, self.bbox_h
        proportions = self.calculate_lock_on_proportion(
            self.target_box_coordinates, bbox_coordinates
        )
        lock_on_status, kilit = self.lock_on_status(
            proportions, self.lock_on_controller, self.kilit_controller
        )
        self.send_lock_on_info(kilit, lock_on_status)
        self.kilit_pub.publish(kilit)

    # Calculate bounding box proportion to the target area
    def calculate_lock_on_proportion(self, target_coordinates, bbox_coordinates):
        bbox_x, bbox_y, bbox_w, bbox_h = bbox_coordinates
        target_x, target_y, target_w, target_h = target_coordinates

        # Calculate bounding box dimensions
        bbox_width = abs(bbox_w - bbox_x)
        bbox_height = abs(bbox_h - bbox_y)
        # Calculate bounding box dimensions
        bbox_width = abs(bbox_w - bbox_x)
        bbox_height = abs(bbox_h - bbox_y)

        # Calculate proportions
        horizontal_proportion = bbox_width / self.width
        vertical_proportion = bbox_height / self.height

        # Check minimum proportion (5%)
        min_proportion = 0.06
        if horizontal_proportion < min_proportion and vertical_proportion < min_proportion:
            rospy.logwarn("Bounding box proportions are too small")
            return 0, 0

        else:
            # Calculate the intersection between bbox and target_bbox
            intersect_x = max(bbox_x, target_x)
            intersect_y = max(bbox_y, target_y)
            intersect_w = min(bbox_w, target_w)
            intersect_h = min(bbox_h, target_h)

            # If there's no intersection, return None
            if intersect_x >= intersect_w or intersect_y >= intersect_h:
                rospy.logwarn("Target is out of locking area")

            # Calculate intersection width and height
            intersect_width = intersect_w - intersect_x
            intersect_height = intersect_h - intersect_y

            try:  # Calculate the proportion of the bounding box that overlaps with the target_bbox
                intersect_horizontal_proportion = intersect_width / bbox_width
                intersect_vertical_proportion = intersect_height / bbox_height
            except ZeroDivisionError as e:
                return 0, 0
            # Return the proportions and bounding box
            return intersect_horizontal_proportion, intersect_vertical_proportion

    def lock_on_status(self, proportions, lock_on_status, kilit):
        if proportions[0] >= 0.06 and proportions[1] >= 0.06:
            start_time = self.timer.start()
            self.elapsed_time = self.timer.elapsed()
            rospy.loginfo(self.elapsed_time)
            if self.elapsed_time >= 4.00:
                lock_on_status = True
                self.timer.reset()
            if self.elapsed_time > 0.00:
                kilit = True
            else:
                kilit = False
        else:
            self.timer.reset()
            lock_on_status = False
        return lock_on_status, kilit

    def send_lock_on_info(self, kilit_msg, lock_on_msg):
        if kilit_msg != True:
            self.timer_check = False
        if kilit_msg is True and self.timer_check is False:
            self.start_time = self.timer.get_current_time()
            self.timer_check = True  # timer started
        tracking_online = rospy.get_param("savasan_gui_node/tracing_online")
        if lock_on_msg == True and tracking_online == 1:
            self.end_time = self.timer.get_current_time()
            try:
                rospy.wait_for_service("send_lock_message")
                send_lock_message = rospy.ServiceProxy(
                    "send_lock_message", sendlock)
                data = Kilitlenme(start_hour=self.start_time[0],
                                  start_min=self.start_time[1],
                                  start_second=self.start_time[2],
                                  start_milisecond=self.start_time[3]//1000,
                                  stop_hour=self.end_time[0],
                                  stop_min=self.end_time[1],
                                  stop_second=self.end_time[2],
                                  stop_milisecond=self.end_time[3]//1000,
                                  otonom=1)
                response = send_lock_message(data)
                rospy.loginfo("KITLENDI VIKTORRRR")
                rospy.set_param("savasan_gui_node/tracing_online", 0)
            except Exception as e:
                rospy.logerr(
                    f"An error occurred while creating lock-on data: {str(e)}")


if __name__ == "__main__":
    rospy.init_node("lock_checker_node", anonymous=True)
    lock_checker_node = Lock_Checker()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Image Processor node")
