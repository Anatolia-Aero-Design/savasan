#!/usr/bin/env python

import rospy
from server_comm.msg import KonumBilgileri
from mavros_msgs.srv import CommandLong, CommandLongRequest
from std_srvs.srv import Trigger, TriggerResponse
from mavros_msgs.srv import CommandInt
from utils import haversine_formula

class Impact_Prevention:
    def __init__(self) -> None:
        self.target_id = rospy.get_param('/gps_navigator/target_id')
    
    def main(self):
        