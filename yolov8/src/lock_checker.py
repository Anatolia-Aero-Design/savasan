#!/usr/bin/env python

import datetime
import logging

import requests
import rospy
from image_processor.msg import Yolo_xywh  # Import the custom message#
from std_msgs.msg import Bool
import time


class Timer:
    def __init__(self) -> None:
        self.start_time = None

    def timer(self):
        if self.start_time is None:
            self.start_time = time.time()

        elapsed_time = time.time() - self.start_time
        return elapsed_time


class Lock_Checker:
    def __init__(self) -> None:
        self.session = requests.Session()
        self.lock_on_controller = None
        self.kilit_controller = None
        self.bbox = self.bbox_x = self.bbox_y = self.bbox_w = self.bbox_h = None

        self.server_url_kilitlenme_bilgisi = rospy.get_param(
            "/comm_node/api/kilitlenme_bilgisi"
        )
        self.xywh_sub = rospy.Subscriber(
            "/yolov8/xywh", Yolo_xywh, self.bbox_callback)

        self.lock_on_pub = rospy.Publisher(
            "/lock_on_status", Bool, queue_size=60)
        self.kilit_pub = rospy.Publisher("/kilit", Bool, queue_size=60)

        # Draw target area
        target_box_x = int(1280 * 0.25)
        target_box_y = int(720 * 0.1)
        target_box_w = int(1280 * 0.75)
        target_box_h = int(720 * 0.9)
        self.target_box_coordinates = (
            target_box_x,
            target_box_y,
            target_box_w,
            target_box_h,
        )

        self.timer = Timer()

    def bbox_callback(self, msg):
        self.bbox = msg
        self.bbox_x = msg.x
        self.bbox_y = msg.y
        self.bbox_w = msg.w
        self.bbox_h = msg.h

    def callback(self):
        bbox_coordinates = self.bbox_x, self.bbox_y, self.bbox_w, self.bbox_h
        proportions = self.calculate_lock_on_proportion(
            self.target_box_coordinates, bbox_coordinates
        )
        lock_on_status, kilit = self.lock_on_status(
            proportions, self.lock_on_controller, self.kilit_controller
        )
        self.send_lock_on_info(kilit, lock_on_status)

    # Calculate bounding box proportion to the target area
    def calculate_lock_on_proportion(self, target_coordinates, bbox_coordinates):
        x_t, y_t, w_t, h_t = target_coordinates
        target_area = abs(w_t - x_t) * abs(h_t - y_t)

        x_d, y_d, w_d, h_d = bbox_coordinates
        bbox_area = abs(w_d - x_d) * abs(h_d - y_d)

        # Initialize proportions with default values
        horizontal_proportion = 0
        vertical_proportion = 0

        if bbox_area <= target_area:
            x_o = max(x_t, x_d)
            y_o = max(y_t, y_d)
            w_o = min(w_t, w_d)
            h_o = min(h_t, h_d)

            overlap_width = max(0, w_o - x_o)
            overlap_height = max(0, h_o - y_o)

            horizontal_proportion = (
                overlap_width / (w_d - x_d) if (w_d - x_d) != 0 else 0
            )
            vertical_proportion = (
                overlap_height / (h_d - y_d) if (h_d - y_d) != 0 else 0
            )
        return horizontal_proportion, vertical_proportion

    def lock_on_status(self, proportions, lock_on_status, kilit):
        if proportions[0] >= 0.93 and proportions[1] >= 0.93:
            elapsed_time = self.timer.timer()
            if elapsed_time >= 4.00:
                lock_on_status = True
            if elapsed_time > 0.00:
                kilit = True
            else:
                kilit = False
        else:
            self.start_time = None
            lock_on_status = False
        return lock_on_status, kilit

    def send_lock_on_info(self, kilit_msg, lock_on_msg):
        if kilit_msg != True:
            self.timer_check = False
        if kilit_msg is True and self.timer_check is False:
            self.start_time = self.get_current_time()
            self.timer_check = True  # timer started
        if lock_on_msg == True and self.kilitlenme_data_sent == False:
            self.end_time = self.get_current_time()
            try:
                data_dict = {
                    "kilitlenmeBaslangicZamani": {
                        "hour": self.start_time[0],
                        "minute": self.start_time[1],
                        "second": self.start_time[2],
                        "millisecond": self.start_time[3]
                    },
                    "kilitlenmeBitisZamani": {
                        "hour": self.end_time[0],
                        "minute": self.end_time[1],
                        "second": self.end_time[2],
                        "millisecond": self.end_time[3]
                    },
                    "otonom_kilitlenme": 1
                }
                response = self.session.post(
                    self.server_url_kilitlenme_bilgisi, json=data_dict)
                if response.status_code == 200:
                    try:
                        response_json = response.json()
                        logging.info(
                            f"Lock-on data sent successfully: {response_json}")
                        rospy.logwarn(f"{data_dict}")
                        self.kilitlenme_data_sent = True
                    except ValueError:
                        logging.error(
                            f"Failed to parse JSON response: {response.text}")
                else:
                    logging.error(
                        f"Failed to send lock-on data, status code: {response.status_code}, response: {response.text}")
            except Exception as e:
                rospy.logerr(
                    f"An error occurred while sending lock-on data: {str(e)}")

    def get_current_time(self):
        now = datetime.now()
        return now.hour, now.minute, now.second, now.microsecond


if __name__ == "__main__":
    rospy.init_node("lock_checker_node", anonymous=True)
    lock_checker_node = Lock_Checker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Image Processor node")