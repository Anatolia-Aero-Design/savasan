#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, BatteryState, NavSatFix # type: ignore
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from message_filters import Subscriber, ApproximateTimeSynchronizer
import requests
import json
import logging
from server_comm.msg import KonumBilgileri, KonumBilgisi
from utils import quaternion_to_euler, calculate_speed, mode_guided

# Configure logging
logging.basicConfig(filename='/home/valvarn/catkin_ws/logs/serverlog.log',level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def callback(imu_msg, battery_msg, rel_altitude_msg, position_msg, speed_msg):
    try:
        # Convert quaternion to euler angles
        roll, pitch, yaw = quaternion_to_euler(imu_msg.orientation.x,
                                               imu_msg.orientation.y,
                                               imu_msg.orientation.z,
                                               imu_msg.orientation.w)

        # Prepare data dictionary
        data_dict = {
            "takim_numarasi": 31,
            "IHA_enlem": position_msg.latitude,
            "IHA_boylam": position_msg.longitude,
            "IHA_irtifa": rel_altitude_msg.data,
            "IHA_yonelme": yaw,
            "IHA_dikilme": pitch,
            "IHA_yatis": roll,
            "IHA_hiz": calculate_speed(speed_msg.twist.linear.x, speed_msg.twist.linear.y, speed_msg.twist.linear.z),
            "IHA_batarya": int(battery_msg.percentage * 100),
            "IHA_otonom": 1
        }

        # Server URL
        server_url = 'http://10.42.0.1:5000/update_data'

        # Send data to the server
        response = requests.post(server_url, json=data_dict)

        
        # Check server response
        logging.info(f"Data sent successfully: {response.json()}")
        if response.status_code == 200:
            logging.info(f"Data sent successfully: {response.json()}")
            parse_and_publish_konumBilgileri(response.json())
        else:
            logging.error(f"Failed to send data, status code: {response.status_code}")

    except Exception as e:
        logging.error(f"An error occurred in callback: {str(e)}")

def parse_and_publish_konumBilgileri(response_json):
    try:
        konumBilgileri_msg = KonumBilgileri()

        # Assuming the response contains konumBilgileri field
        konumBilgileri_data = response_json.get("konumBilgileri", [])
        for item in konumBilgileri_data:
            konum_bilgisi = KonumBilgisi()
            konum_bilgisi.IHA_boylam = item["IHA_boylam"]
            konum_bilgisi.IHA_dikilme = item["IHA_dikilme"]
            konum_bilgisi.IHA_enlem = item["IHA_enlem"]
            konum_bilgisi.IHA_hiz = item["IHA_hiz"]
            konum_bilgisi.IHA_irtifa = item["IHA_irtifa"]
            konum_bilgisi.IHA_yatis = item["IHA_yatis"]
            konum_bilgisi.IHA_yonelme = item["IHA_yonelme"]
            konum_bilgisi.IHA_zamanfarki = item["IHA_zamanfarki"]
            konum_bilgisi.takim_numarasi = item["takim_numarasi"]

            konumBilgileri_msg.konumBilgileri.append(konum_bilgisi)

        konum_pub.publish(konumBilgileri_msg)
        logging.info("Published konumBilgileri message.")

    except Exception as e:
        logging.error(f"An error occurred while parsing and publishing konumBilgileri: {str(e)}")

def synchronize_topics():
    try:
        rospy.init_node('sync_node', anonymous=True)

        # Define subscribers
        imu_sub = Subscriber('/mavros/imu/data', Imu)
        battery_sub = Subscriber('/mavros/battery', BatteryState)
        rel_altitude_sub = Subscriber('/mavros/global_position/rel_alt', Float64)
        position_sub = Subscriber('/mavros/global_position/global', NavSatFix)
        speed_sub = Subscriber('/mavros/local_position/velocity_local', TwistStamped)
        state_sub = Subscriber('/mavros/state', State)

        # ApproximateTimeSynchronizer to synchronize messages based on timestamps
        sync = ApproximateTimeSynchronizer(
            [imu_sub, battery_sub, rel_altitude_sub, position_sub, speed_sub],
            queue_size=20,
            slop=0.005,  # Adjust this parameter based on your message timestamp tolerances
            allow_headerless=True
        )
        sync.registerCallback(callback)
        
        # Initialize the konum publisher
        global konum_pub
        konum_pub = rospy.Publisher('konum_bilgileri', KonumBilgileri, queue_size=10)

        logging.info("Synchronize topics node started.")
        rospy.spin()

    except Exception as e:
        logging.exception(f"An error occurred while initializing the node: {str(e)}")

if __name__ == '__main__':
    try:
        synchronize_topics()
    except rospy.ROSInterruptException:
        logging.info("ROS node interrupted.")
    except Exception as e:
        logging.exception(f"An error occurred in the main loop: {str(e)}")
