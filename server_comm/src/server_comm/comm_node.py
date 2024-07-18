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
import datetime
from datetime import datetime
import time 
from image_processor.msg import Yolo_xywh 

class Comm_Node:
    
    def __init__(self):

        rospy.init_node('comm_node', anonymous=True)
    # Get server APIs from sim.launch params
        from message_filters import Subscriber
        self.server_url_telemetri_gonder = rospy.get_param('/comm_node/api/telemetri_gonder')
        self.server_url_kilitlenme_bilgisi = rospy.get_param('/comm_node/api/kilitlenme_bilgisi')
        self.server_url_sunucusaati = rospy.get_param('/comm_node/api/sunucusaati')
        
        
        # Initialize Subscribers
        self.imu_sub = Subscriber('/mavros/imu/data', Imu)
        self.battery_sub = Subscriber('/mavros/battery', BatteryState)
        self.rel_altitude_sub = Subscriber('/mavros/global_position/rel_alt', Float64)
        self.position_sub = Subscriber('/mavros/global_position/global', NavSatFix)
        self.speed_sub = Subscriber('/mavros/local_position/velocity_local', TwistStamped)
        self.state_sub = Subscriber('/mavros/state', State)
        self.lock_on_sub = Subscriber('/lock_on_status', Bool) 
        self.kilit_sub = Subscriber('/kilit', Bool) 
        self.bbox_sub = Subscriber("/yolov8/xywh", Yolo_xywh)

        self.bbox_x = 0.0
        self.bbox_y = 0.0
        self.bbox_w = 0.0
        self.bbox_h = 0.0
        

        # Configure logging
        logging.basicConfig(filename='/home/poyraz/catkin_ws/logs/serverlog.log',level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

        # Initialize Time sync
        self.sync = ApproximateTimeSynchronizer(
            [self.imu_sub, self.battery_sub, self.rel_altitude_sub, self.position_sub, self.speed_sub,self.state_sub, self.lock_on_sub, self.kilit_sub,self.bbox_sub],
            queue_size=20,
            slop=0.005,  # Adjusted slop for better tolerance
            allow_headerless=True
        )
        self.sync.registerCallback(self.callback)
        
        # Initialize Publisers
        self.server_time_pub = rospy.Publisher('/server_time', String, queue_size=10)
        self.konum_pub = rospy.Publisher('konum_bilgileri', KonumBilgileri, queue_size=10)
        
       

    def callback(self, imu_msg, battery_msg, rel_altitude_msg, position_msg, speed_msg, state_msg, kilit_msg, bbox_msg):

        if state_msg.mode == 'AUTO' or state_msg.mode.startswith('GUIDED'):
                IHA_otonom = 1  # Set to 1 for autonomous modes
        else:
                IHA_otonom = 0  # Set to 0 for manual modes

        iha_kilitlenme = 1 if kilit_msg else 0

        self.bbox_x = bbox_msg.bbox_x
        self.bbox_y = bbox_msg.bbox_y
        self.bbox_w = bbox_msg.bbox_w
        self.bbox_h = bbox_msg.bbox_h

        rospy.loginfo(f"Received bbox data: x={self.bbox_x}, y={self.bbox_y}, w={self.bbox_w}, h={self.bbox_h}")

        

        try:
            
            server_time = self.get_server_time()
            
            # Set the server time as a ROS parameter
            rospy.set_param('/server_time', server_time)
            
            # Convert quaternion to euler angles
            roll, pitch, yaw = quaternion_to_euler(imu_msg.orientation.x,
                                                imu_msg.orientation.y,
                                                imu_msg.orientation.z,
                                                imu_msg.orientation.w)

            # Prepare data dictionary
            data_dict = {
                "takim_numarasi": 69,
                "IHA_enlem": position_msg.latitude,
                "IHA_boylam": position_msg.longitude,
                "IHA_irtifa": rel_altitude_msg.data,
                "IHA_yonelme": yaw,
                "IHA_dikilme": pitch,
                "IHA_yatis": roll,
                "IHA_hiz": calculate_speed(speed_msg.twist.linear.x, speed_msg.twist.linear.y, speed_msg.twist.linear.z),
                "IHA_batarya": int(battery_msg.percentage * 100),
                "IHA_otonom": IHA_otonom,
                "iha_kilitlenme": iha_kilitlenme ,
                "hedef_merkez_X": self.bbox_x,
                "hedef_merkez_Y": self.bbox_y,
                "hedef_genislik": self.bbox_w,
                "hedef_yukseklik": self.bbox_h,
                "sunucusaati": server_time 
            }

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

            self.konum_pub.publish(konumBilgileri_msg)
            logging.info("Published konumBilgileri message.")

        except Exception as e:
            logging.error(f"An error occurred while parsing and publishing konumBilgileri: {str(e)}")


    def send_lock_on_info(self, kilit_msg, lock_on_msg):

        global start_time, end_time, kilit_prev

        def get_current_time():
            now = datetime.now()
            return now.hour, now.minute, now.second, now.microsecond

        while True:
            start_time = None
            end_time = None
            kilit_prev = None  
            current_time = datetime.now()

            if kilit_msg is None and kilit_prev is None:
                pass

            elif kilit_prev is None and kilit_msg is not None:

                kilit_prev = kilit_msg

            elif kilit_prev is True and kilit_msg is True or kilit_prev is False and kilit_msg is True:

                    start_time = get_current_time()


            elif kilit_prev is True and kilit_msg is False:

                end_time = get_current_time()

                if lock_on_msg:
                    try:
                        data_dict = {
                                "kilitlenmeBaslangicZamani": {
                                "hour": start_time[0],
                                "minute": start_time[1],
                                "second": start_time[2],
                                "millisecond": start_time[3]
                            },
                            "kilitlenmeBitisZamani": {
                                "hour": end_time[0],
                                "minute": end_time[1],
                                "second": end_time[2],
                                "millisecond": end_time[3]
                            },
                            "otonom_kilitlenme": 1
                        }
                    


                        server_url = 'http://10.42.0.1:5000/api/kilitlenme_bilgisi'
                        response = requests.post(server_url, json=data_dict)
                        if response.status_code == 200:
                            logging.info(f"Lock-on data sent successfully: {response.json()}")
                        else:
                            logging.error(f"Failed to send lock-on data, status code: {response.status_code}")

                    except Exception as e:
                        logging.error(f"An error occurred while sending lock-on data: {str(e)}")

            kilit_prev = kilit_msg  # Update previous status
            time.sleep(0.1)


    def get_server_time(self):
        try:
            response = requests.get(self.server_url_sunucusaati)
            if response.status_code == 200:
                server_time = response.json().get('sunucusaati')
                logging.info(f"Server time retrieved successfully: {server_time}")
                formatted_time_dict = {
                "gun": server_time["gun"],
                "saat": server_time["saat"],
                "dakika": server_time["dakika"],
                "saniye": server_time["saniye"],
                "milisaniye": server_time["milisaniye"]
                }
                self.server_time_pub.publish(json.dumps(formatted_time_dict))
                return formatted_time_dict
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
