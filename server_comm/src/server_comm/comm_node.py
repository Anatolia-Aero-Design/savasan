#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Imu, BatteryState, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
import requests
import json
import logging
from server_comm.msg import KonumBilgileri, KonumBilgisi
from utils import quaternion_to_euler, calculate_speed, mode_guided
from datetime import datetime
import time
import threading
from image_processor.msg import Yolo_xywh

class Comm_Node:
    def __init__(self):
        # Get server APIs from sim.launch params
        self.server_url_telemetri_gonder = rospy.get_param('/comm_node/api/telemetri_gonder')
        self.server_url_kilitlenme_bilgisi = rospy.get_param('/comm_node/api/kilitlenme_bilgisi')
        self.server_url_sunucusaati = rospy.get_param('/comm_node/api/sunucusaati')
        self.server_url_qr_bilgisi = rospy.get_param('/comm_node/api/qr_bilgisi')

        self.imu = None
        self.battery = None
        self.rel_alt = None
        self.position = None
        self.speed = None
        self.state = None
        self.lock_on = None
        self.kilit = None
        self.qr_data = None

        try:
            # Initialize Subscribers
            self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
            self.battery_sub = rospy.Subscriber('/mavros/battery', BatteryState, self.battery_callback)
            self.rel_altitude_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.rel_altitude_callback)
            self.position_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
            self.speed_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.speed_callback)
            self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
            self.lock_on_sub = rospy.Subscriber('/lock_on_status', Bool, self.lock_on_callback)
            self.kilit_sub = rospy.Subscriber('/kilit', Bool, self.kilit_callback)
            self.bbox_sub = rospy.Subscriber("/yolov8/xywh", Yolo_xywh, self.bbox_callback)
            self.qr_sub = rospy.Subscriber("/qr_code_data", String, self.qr_callback)
        
        except Exception as e:
            print("error")
            logging.error(f"An error occurred in process_data: {str(e)}")
        
        finally:
            self.process_data()

        # Initialize Publishers
        self.server_time_pub = rospy.Publisher('/server_time', String, queue_size=10)
        self.konum_pub = rospy.Publisher('/konum_bilgileri', KonumBilgileri, queue_size=10)

        
        self.bbox = None
        self.bbox_x = None
        self.bbox_y = None
        self.bbox_w = None
        self.bbox_h = None
        
        self.start_time = None
        self.end_time = None
        self.kilit_prev = None

        # Configure logging
        logging.basicConfig(filename='/home/poyrazzo/catkin_ws/logs/serverlog.log', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
        
    def qr_callback(self,msg):
        self.qr_data = msg.data
        self.publish_qr()
        
    def lock_on_callback(self, msg):
        self.lock_on = msg.data
        rospy.loginfo(f"Received lock_on_status: {msg.data}")
        logging.info(f"Received lock_on_status: {msg.data}")
        self.process_data()
    
    def kilit_callback(self, msg):
        self.kilit = msg.data
        rospy.loginfo(f"Received kilit: {msg.data}")
        logging.info(f"Received kilit: {msg.data}")
        self.process_data()

        self.lock_on_thread = threading.Thread(target=self.send_lock_on_info)
        self.lock_on_thread.start()
    
    def imu_callback(self, msg):
        self.imu = msg
        self.process_data()

    def battery_callback(self, msg):
        self.battery = msg
        self.process_data()

    def rel_altitude_callback(self, msg):
        self.rel_alt = msg
        self.process_data()

    def position_callback(self, msg):
        self.position = msg
        self.process_data()

    def speed_callback(self, msg):
        self.speed = msg
        self.process_data()

    def state_callback(self, msg):
        self.state = msg
        self.process_data()

    

    def bbox_callback(self, msg):
        self.bbox = msg
        self.bbox_x = msg.x
        self.bbox_y = msg.y
        self.bbox_w = msg.w
        self.bbox_h = msg.h
        rospy.loginfo(f"Received bbox data: x={self.bbox_x}, y={self.bbox_y}, w={self.bbox_w}, h={self.bbox_h}")
        logging.info(f"Received bbox data: x={self.bbox_x}, y={self.bbox_y}, w={self.bbox_w}, h={self.bbox_h}")
        self.process_data()

    def process_data(self):
        if not (self.imu and self.battery and self.rel_alt and self.position and self.speed and self.state):
            return

        if self.state.mode == 'AUTO' or self.state.mode.startswith('GUIDED'):
            IHA_otonom = 1  # Set to 1 for autonomous modes
        else:
            IHA_otonom = 0  # Set to 0 for manual modes

        iha_kilitlenme = 1 if self.kilit else 0

        try:
            

            # Convert quaternion to euler angles
            roll, pitch, yaw = quaternion_to_euler(self.imu.orientation.x,
                                                   self.imu.orientation.y,
                                                   self.imu.orientation.z,
                                                   self.imu.orientation.w)

            # Prepare data dictionary
            data_dict = {
                "takim_numarasi": 31,
                "IHA_enlem": self.position.latitude,
                "IHA_boylam": self.position.longitude,
                "IHA_irtifa": self.rel_alt.data,
                "IHA_yonelme": yaw,
                "IHA_dikilme": pitch,
                "IHA_yatis": roll,
                "IHA_hiz": calculate_speed(self.speed.twist.linear.x, self.speed.twist.linear.y, self.speed.twist.linear.z),
                "IHA_batarya": int(self.battery.percentage * 100),
                "IHA_otonom": IHA_otonom,
                "iha_kilitlenme": iha_kilitlenme,
                "hedef_merkez_X": self.bbox_x,
                "hedef_merkez_Y": self.bbox_y,
                "hedef_genislik": self.bbox_w,
                "hedef_yukseklik": self.bbox_h,
                "sunucusaati": self.get_server_time()
            }
            self.publish_qr()
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
            logging.error(f"An error occurred in process_data: {str(e)}")

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

    def get_current_time(self):
        now = datetime.now()
        return now.hour, now.minute, now.second, now.microsecond
        
    def send_lock_on_info(self):
        while not rospy.is_shutdown():
            if self.kilit is None and self.kilit_prev is None:
                pass

            elif self.kilit_prev is None and self.kilit is not None:
                self.kilit_prev = self.kilit

            elif self.kilit_prev is True and self.kilit is True or self.kilit_prev is False and self.kilit is True:
                self.start_time = self.get_current_time()

            elif self.kilit_prev is True and self.kilit is False and self.lock_on is True:
                self.end_time = self.get_current_time()

                if self.lock_on:
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
  
                        print("Lock-on Info Dictionary:")
                        print(json.dumps(data_dict, indent=4))

                        response = requests.post(self.server_url_kilitlenme_bilgisi, json=data_dict)
                        if response.status_code == 200:
                            logging.info(f"Lock-on data sent successfully: {response.json()}")
                        else:
                            logging.error(f"Failed to send lock-on data, status code: {response.status_code}")

                    except Exception as e:
                        logging.error(f"An error occurred while sending lock-on data: {str(e)}")

            self.kilit_prev = self.kilit  # Update previous status
            time.sleep(0.1)
            
            
    def publish_qr(self):
        #if self.qr_data:
                try:
                    start_time = rospy.get_param('/kamikazeBaslangicZamani', [0, 0, 0, 0])
                    end_time = rospy.get_param('/kamikazeBitisZaman', [0, 0, 0, 0])
                    
                    rospy.loginfo(f"Start Time Retrieved: {start_time}")
                    rospy.loginfo(f"End Time Retrieved: {end_time}")
                    
                    data_dict = {
                        "kamikazeBaslangicZamani": {
                            "hour": start_time[0],
                            "minute": start_time[1],
                            "second": start_time[2],
                            "millisecond": start_time[3]
                        },
                        "kamikazeBitisZaman": {
                            "hour": end_time[0],
                            "minute": end_time[1],
                            "second": end_time[2],
                            "millisecond": end_time[3]
                        },
                        "qrMetni ": self.qr_data if self.qr_data else "No QR Data" 
                    }
                    
                    print("Correction Timing Dictionary:")
                    print(json.dumps(data_dict, indent=4))
                    # Here you could send the data, save it to a file, etc.
                    
                    response = requests.post(self.server_url_qr_bilgisi, json=data_dict)
                    if response.status_code == 200:
                        logging.info(f"Lock-on data sent successfully: {response.json()}")
                    else:
                        logging.error(f"Failed to send lock-on data, status code: {response.status_code}")
                except KeyError:
                    rospy.logerr("Failed to retrieve start or end time from ROS parameters.")
        #else:
        #       rospy.logerr("no qr_data available")


    def get_server_time(self):
        try:
            logging.info(f"Requesting server time from {self.server_url_sunucusaati}")
            response = requests.get('http://127.0.0.1:5000/api/sunucusaati')
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
                formatted_time_str = json.dumps(formatted_time_dict)
                self.server_time_pub.publish(formatted_time_str)
                return formatted_time_dict
            else:
                logging.error(f"Failed to retrieve server time, status code: {response.status_code}")
                return None
        except Exception as e:
            logging.error(f"An error occurred while retrieving server time: {str(e)}")
            return None
        
"""    def get_coordinates(self):
        try:
            response = requests.get(self.server_url_qr_koordinati)
            if response.status_code == 200:
                json_data = response.json()
                self.qr_pose_pub.publish(json_data)
            else:
                rospy.logwarn(f"Error: Unable to fetch data from URL. Status code: {response.status_code}")
        except Exception as e:
            rospy.logerr(f"Error: {e}")"""


if __name__ == '__main__':
    rospy.init_node('comm_node', anonymous=True)
    comm_node = Comm_Node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        logging.info("ROS node interrupted.")
    except Exception as e:
        logging.error(f"An error occurred in the main loop: {str(e)}")