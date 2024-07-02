#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, BatteryState, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from message_filters import Subscriber, ApproximateTimeSynchronizer
import requests
import logging
from server_comm.msg import KonumBilgileri, KonumBilgisi, ServerTime
from utils import quaternion_to_euler, calculate_speed

# Configure logging
logging.basicConfig(filename='/home/valvarn/catkin_ws/logs/serverlog.log', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

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

        logging.info("Prepared data dictionary for server update.")
        
        server_url_update = rospy.get_param('/update_data')
        get_server_time_url = rospy.get_param('/get_server_time')
        
        # Send data to the server
        response = requests.post(server_url_update, json=data_dict)
        logging.info(f"Sent data to server: {data_dict}")

        # Fetch server time
        server_time = fetch_server_time(get_server_time_url)
        if server_time:
            logging.info(f"Server time: {server_time}")
            publish_server_time(server_time)
        else:
            logging.warning("Failed to fetch server time.")
            
        # Check server response
        if response.status_code == 200:
            logging.info(f"Data sent successfully: {response.json()}")
            parse_and_publish_konumBilgileri(response.json())
        else:
            logging.error(f"Failed to send data, status code: {response.status_code}")

    except Exception as e:
        logging.error(f"An error occurred in callback: {str(e)}")

def fetch_server_time(server_url):
    try:
        response = requests.get(server_url)
        response.raise_for_status()  # Raise an HTTPError on bad responses
        data_dict = response.json()
        sunucusaati = data_dict.get('sunucusaati', {})
        return sunucusaati
    except requests.exceptions.RequestException as e:
        logging.error(f"Failed to fetch data from server: {e}")
        return None 

def publish_server_time(server_time):
    try:
        server_time_msg = ServerTime()
        server_time_msg.gun = server_time.get('gun', 0)
        server_time_msg.saat = server_time.get('saat', 0)
        server_time_msg.dakika = server_time.get('dakika', 0)
        server_time_msg.saniye = server_time.get('saniye', 0)
        server_time_msg.milisaniye = server_time.get('milisaniye', 0)
        server_time_pub.publish(server_time_msg)
        logging.info("Published server time message.")
    except Exception as e:
        logging.error(f"An error occurred while publishing server time: {str(e)}")

def parse_and_publish_konumBilgileri(response_json):
    try:
        konumBilgileri_msg = KonumBilgileri()
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
        
        imu_sub = Subscriber('/mavros/imu/data', Imu)
        battery_sub = Subscriber('/mavros/battery', BatteryState)
        rel_altitude_sub = Subscriber('/mavros/global_position/rel_alt', Float64)
        position_sub = Subscriber('/mavros/global_position/global', NavSatFix)
        speed_sub = Subscriber('/mavros/local_position/velocity_local', TwistStamped)
        state_sub = Subscriber('/mavros/state', State)

        sync = ApproximateTimeSynchronizer(
            [imu_sub, battery_sub, rel_altitude_sub, position_sub, speed_sub],
            queue_size=20,
            slop=0.005,  # Adjusted slop for better tolerance
            allow_headerless=True
        )
        sync.registerCallback(callback)
        
        global konum_pub, server_time_pub
        konum_pub = rospy.Publisher('konum_bilgileri', KonumBilgileri, queue_size=10)
        server_time_pub = rospy.Publisher('get_server_time', ServerTime, queue_size=10)

        logging.info("Synchronize topics node started.")
        rospy.spin()

    except Exception as e:
        logging.error(f"An error occurred while initializing the node: {str(e)}")

if __name__ == '__main__':
    try:
        synchronize_topics()
    except rospy.ROSInterruptException:
        logging.info("ROS node interrupted.")
    except Exception as e:
        logging.error(f"An error occurred in the main loop: {str(e)}")
