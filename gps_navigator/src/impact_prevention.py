#!/usr/bin/env python

import rospy
from server_comm.msg import KonumBilgileri
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
import math
from mavros_msgs.srv import CommandInt
from src.utils import haversine_formula


class Impact_Prevention:
    def __init__(self) -> None:
        self.target_id = None
        self.position = None
        self.rel_alt = None

        self.target_id = rospy.get_param('/gps_navigator/target_id')
        self.gps_sub = rospy.Subscriber('/konum_bilgileri', KonumBilgileri)
        self.position_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.position_callback
        )
        self.rel_altitude_sub = rospy.Subscriber(
            "/mavros/global_position/rel_alt", Float64, self.rel_altitude_callback
        )

    def rel_altitude_callback(self, msg):
        self.rel_alt = msg

    def position_callback(self, msg):
        self.position = msg

    def distance_calculator(self, msg: KonumBilgileri):
        for konum in msg.konumBilgileri:
            if konum.takim_numarasi == self.target_id:
                latitude = konum.IHA_enlem
                longitude = konum.IHA_boylam
                altitude = konum.IHA_irtifa
                distance = haversine_formula(
                    latitude, longitude, self.position.latitude, self.position.longitude)
                alt_diff = abs(altitude - self.rel_alt)

                distance = math.sqrt(distance**2 + alt_diff**2)
                return distance

    def main(self):
        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not rospy.is_shutdown():
            while self.target_id is not None:
                distance = self.distance_calculator()
                if distance is not None and distance <= 30:
                    rospy.wait_for_service('stop_navigation')
                    stop_service = rospy.ServiceProxy(
                        "stop_navigation", Trigger)
                    response = stop_service()
                    if response == 1:
                        rospy.logwarn(
                            f"GPS navigation stopped due to distance: {distance} meters")
            rate.sleep()  # Sleep to maintain the loop rate


if __name__ == '__main__':
    try:
        rospy.init_node('impact_prevention_node', anonymous=True)
        impact_prevention = Impact_Prevention()
        impact_prevention.main()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS Interrupt: {e}")