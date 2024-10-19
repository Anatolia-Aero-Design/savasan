#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList, Waypoint, WaypointReached
import datetime


from mavros_msgs.srv import WaypointPush, WaypointClear, CommandLong, WaypointClear
import math
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
import utilities as utils
from geometry_msgs.msg import PoseStamped
from server_comm.srv import sendqr
from server_comm.msg import Qr
from QR_node import QR_Node
import numpy as np
import time



if __name__ == '__main__':
    rospy.init_node("test", anonymous=True)
    rospy.wait_for_service("/start_Qr")
    qr_start = rospy.ServiceProxy('/start_Qr', Trigger)
    response = qr_start()
    # TODO: RESPONSE CHECK CAN BE ADD
    start_time = rospy.Time.now()
    rospy.loginfo(f"KAMIKAZE STARTED AT: {start_time}")
    time.sleep(3)
   
    rospy.wait_for_service("/stop_Qr")
    qr_stop = rospy.ServiceProxy('/stop_Qr', Trigger)
    response = qr_stop()
    stop_time = rospy.Time.now()
    qr_text = response.message
    rospy.wait_for_service("/send_qr_message")
    secs = start_time.to_sec()
    time_in_datetime = datetime.datetime.fromtimestamp(secs)

    start_hour = time_in_datetime.hour
    start_min = time_in_datetime.minute
    start_second = time_in_datetime.second
    start_millisecond = int(
        time_in_datetime.microsecond / 1000)

    secs = stop_time.to_sec()
    time_in_datetime = datetime.datetime.fromtimestamp(secs)
    stop_hour = time_in_datetime.hour
    stop_min = time_in_datetime.minute
    stop_second = time_in_datetime.second
    stop_millisecond = int(time_in_datetime.microsecond / 1000)

    send_lock_message = rospy.ServiceProxy(
        '/send_qr_message', sendqr)
    rospy.loginfo(f"KAMIKAZE sstop AT: {start_time}")
    response = send_lock_message(Qr(start_hour=start_hour,
                                    start_min=start_min,
                                    start_second=start_second,
                                    start_milisecond=start_millisecond,
                                    stop_hour=stop_hour,
                                    stop_min=stop_min,
                                    stop_second=stop_second,
                                    stop_milisecond=stop_millisecond,
                                    qr_text=qr_text))

