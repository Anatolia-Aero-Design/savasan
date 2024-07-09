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
from server_comm.msg import KonumBilgileri, KonumBilgisi
from utils import quaternion_to_euler, calculate_speed

class Comm_Node:
    def __init__(self):
        # Get server APIs from sim.launch params
        self.server_url_telemetri_gonder = rospy.get_param('/comm_node/api/telemetri_gonder')
        self.server_url_sunucusaati = rospy.get_param('/comm_node/api/sunucusaati')
        
        # Initialize Subscribers
        self.imu_sub = Subscriber('/mavros/imu/data', Imu)
        self.battery_sub = Subscriber('/mavros/battery', BatteryState)
        self.rel_altitude_sub = Subscriber('/mavros/global_position/rel_alt', Float64)
        self.position_sub = Subscriber('/mavros/global_position/global', NavSatFix)
        self.speed_sub = Subscriber('/mavros/local_position/velocity_local', TwistStamped)
        self.state_sub = Subscriber('/mavros/state', State)

        # Configure logging
        logging.basicConfig(filename='/home/valvarn/catkin_ws/logs/serverlog.log', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
        
        # Initialize Time sync
        self.sync = ApproximateTimeSynchronizer(
            [self.imu_sub, self.battery_sub, self.rel_altitude_sub, self.position_sub, self.speed_sub],
            queue_size=20,
            slop=0.005,  # Adjusted slop for better tolerance
            allow_headerless=True
        )
        self.sync.registerCallback(self.callback)
        
        # Initialize Publishers
        self.konum_pub = rospy.Publisher('konum_bilgileri', KonumBilgileri, queue_size=10)
      

    def callback(self, imu_msg, battery_msg, rel_altitude_msg, position_msg, speed_msg):
        try:
            # Get server time during initialization
            server_time = self.get_server_time()
            
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
                "IHA_otonom": 1,
                "sunucusaati": server_time  # Add server time to data
            }
            print(data_dict["sunucusaati"])
            logging.info("Prepared data dictionary for server update.")
            
            # Send data to the server
            response = requests.post(self.server_url_telemetri_gonder, json=data_dict)
            logging.info(f"Sent data to server: {data_dict}")
                
            # Check server response
            if response.status_code == 200:
                logging.info(f"Data sent successfully: {response.json()}")
                self.parse_and_publish_konumBilgileri(response.json())
            else:
                logging.error(f"Failed to send data, status code: {response.status_code}")
        
        except Exception as e:
            logging.error(f"An error occurred in callback: {str(e)}")

    def parse_and_publish_konumBilgileri(self, response_json):
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

            self.konum_pub.publish(konumBilgileri_msg)  
            logging.info("Published konumBilgileri message.")

        except Exception as e:
            logging.error(f"An error occurred while parsing and publishing konumBilgileri: {str(e)}")
    
    def get_server_time(self):
        try:
            response = requests.get(self.server_url_sunucusaati)
            if response.status_code == 200:
                server_time = response.json().get('sunucusaati')
                logging.info(f"Server time retrieved successfully: {server_time}")
                return server_time
            else:
                logging.error(f"Failed to retrieve server time, status code: {response.status_code}")
                return None
        except Exception as e:
            logging.error(f"An error occurred while retrieving server time: {str(e)}")
            return None

if __name__ == '__main__':
    rospy.init_node('comm_node', anonymous=True)
    comm_node = Comm_Node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        logging.info("ROS node interrupted.")
    except Exception as e:
        logging.error(f"An error occurred in the main loop: {str(e)}")