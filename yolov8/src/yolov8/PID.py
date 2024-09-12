#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Imu
from mavros_msgs.msg import AttitudeTarget
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from std_srvs.srv import Empty, EmptyResponse
from image_processor.msg import Yolo_xywh  # Import the custom message#
import cv2
import time
import math
import numpy as np

# Deprecated node // left for future development purposes


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output


def calculate_yaw_pitch_roll(image, bounding_box, image_shape):
    img_height, img_width = image_shape[:2]

    # Use the attributes of the Yolo_xywh object directly
    box_x = bounding_box.x
    box_y = bounding_box.y
    box_width = bounding_box.w
    box_height = bounding_box.h

    # Calculate the center of the bounding box
    box_center_x = box_x + box_width / 2
    box_center_y = box_y + box_height / 2

    # Calculate offsets from the image center
    offset_x = box_center_x - img_width / 2
    offset_y = box_center_y - img_height / 2

    # Normalize offsets to [-1, 1] range
    norm_offset_x = offset_x / (img_width / 2)
    norm_offset_y = offset_y / (img_height / 2)

    # Calculate yaw and pitch
    yaw = norm_offset_x * 45  # Assume max yaw is ±45 degrees
    pitch = -norm_offset_y * 45  # Assume max pitch is ±45 degrees

    # Roll is not determined by bounding box in this simple example
    roll = 0

    target_box_x = int(1280 * 0.25)
    target_box_y = int(720 * 0.1)
    target_box_w = int(1280 * 0.75)
    target_box_h = int(720 * 0.9)

    bbox_x, bbox_y, bbox_w, bbox_h = box_x, box_y, box_width, box_height
    center_target_x = (target_box_w + target_box_x) // 2
    center_target_y = (target_box_h + target_box_y) // 2
    center_x = (bbox_w + bbox_x) // 2
    center_y = (bbox_h + bbox_y) // 2
    if bbox_x != 0 or bbox_y != 0 or bbox_w != 0 or bbox_h != 0:
        cv2.rectangle(image, (bbox_x, bbox_y), (bbox_w, bbox_h),
                      (0, 0, 255), 2)  # Updated to draw correctly
        cv2.arrowedLine(image, (center_target_x, center_target_y),
                        (center_x, center_y), (0, 0, 255), 2)
        cv2.rectangle(image, (target_box_x, target_box_y),
                      (target_box_w, target_box_h), (0, 255, 0), 2)
    return yaw, pitch, roll


def perform_correction(self, yaw_error, pitch_error, dt):
    """Perform correction based on error."""

    roll_correction = self.yaw_pid.compute(yaw_error, dt)
    pitch_correction = self.pitch_pid.compute(pitch_error, dt)

    roll_correction = max(min(roll_correction, 20), -20)
    pitch_correction = max(min(pitch_correction, 25), -70)

    pitch = pitch_correction
    roll = roll_correction

    # self.attitude_controller.set_attitude(roll, pitch, 0, 0)
    # rospy.loginfo(f"roll correction: {roll_correction},")
    # rospy.loginfo(f"pitch correction: {-pitch_correction},")
    return roll, pitch


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


def draw_yaw_pitch_roll_text(image, yaw, pitch, roll):
    # Draw yaw, pitch, and roll values on the image
    cv2.putText(image, f"Yaw: {yaw:.2f} deg", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(image, f"Pitch: {pitch:.2f} deg", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(image, f"Roll: {roll:.2f} deg", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)


if __name__ == '__main__':
    rospy.init_node('PID', anonymous=True)
    pid = PID()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLOv8 tracking node")
