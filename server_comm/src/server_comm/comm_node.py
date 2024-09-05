#!/usr/bin/env python


import rospy
from flask import session
from sensor_msgs.msg import Imu, BatteryState, NavSatFix, TimeReference
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool, String
from std_srvs.srv import Trigger
from server_comm.msg import Kilitlenme
from server_comm.srv import sendlock, sendlockResponse,sendqr,sendqrResponse


from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
import requests
import json
import logging
from server_comm.msg import KonumBilgileri, KonumBilgisi

from utils import quaternion_to_euler, calculate_speed, unix_to_utc_formatted

from image_processor.msg import Yolo_xywh


class Comm_Node:
    def __init__(self):
        self.session = requests.Session()
        self.base_url = "http://savasaniha.baykartech.com/api"
        self.username = "estuanatolia"
        self.password = "2Eqtm3v3ZJ"
        self.team_number = None
        self.header  = {"Content-Type": "application/json", "Accept": "application/json"}

        # Get server APIs from sim.launch params
        self.server_url_telemetri_gonder = rospy.get_param(
            "/comm_node/api/telemetri_gonder"
        )
        self.server_url_sunucusaati = rospy.get_param(
            "/comm_node/api/sunucusaati")
        self.server_url_kamikaze_bilgisi = rospy.get_param(
            "/comm_node/api/kamikaze_bilgisi"
        )

        self.abort_service = rospy.Service(
            "stop_kamikaze", Trigger, self.qr_check)
        
        self.send_lock_message = rospy.Service("send_lock_message",sendlock,self.send_lock_callback)
        self.send_qr_message = rospy.Service("send_qr_message",sendqr,self.send_qr_callback)
        
        self.imu = None
        self.battery = None
        self.rel_alt = None
        self.position = None
        self.speed = None
        self.state = None
        self.qr_data = None
        self.qr_published = False
        self.fcu_time = None
        self.fcu_time_nsecs = None
        self.kilit = None

        self.bbox = None
        self.bbox_x = None
        self.bbox_y = None
        self.bbox_w = None
        self.bbox_h = None

        self.start_time = None
        self.end_time = None

        try:
            # Initialize Subscribers
            self.imu_sub = rospy.Subscriber(
                "/mavros/imu/data", Imu, self.imu_callback)
            self.battery_sub = rospy.Subscriber(
                "/mavros/battery", BatteryState, self.battery_callback
            )
            self.rel_altitude_sub = rospy.Subscriber(
                "/mavros/global_position/rel_alt", Float64, self.rel_altitude_callback
            )
            self.position_sub = rospy.Subscriber(
                "/mavros/global_position/global", NavSatFix, self.position_callback
            )
            self.speed_sub = rospy.Subscriber(
                "/mavros/local_position/velocity_local",
                TwistStamped,
                self.speed_callback,
            )
            self.state_sub = rospy.Subscriber(
                "/mavros/state", State, self.state_callback
            )
            self.bbox_sub = rospy.Subscriber(
                "/yolov8/xywh", Yolo_xywh, self.bbox_callback
            )
            self.qr_sub = rospy.Subscriber(
                "/qr_code_data", String, self.qr_callback)
            self.fcu_time_sub = rospy.Subscriber(
                "/mavros/time_reference", TimeReference, self.fcu_time_callback
            )
            self.kilit_sub = rospy.Subscriber(
                "/kilit", Bool, self.kilit_callback)

        except Exception as e:
            print("error")
            logging.error(f"An error occurred in process_data: {str(e)}")

        # Initialize Publishers
        self.server_time_pub = rospy.Publisher(
            "/server_time", String, queue_size=10)
        self.konum_pub = rospy.Publisher(
            "/konum_bilgileri", KonumBilgileri, queue_size=10
        )

        # Configure logging
        logging.basicConfig(
            filename="/home/valvarn/catkin_ws/logs/serverlog.log",
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - %(message)s",
        )
        
    def send_lock_callback(self,data_calback):

        data = {"kilitlenmeBaslangicZamani":
                    {"saat": data_calback.data.start_hour,
                    "dakika": data_calback.data.start_min,
                    "saniye": data_calback.data.start_second,
                    "milisaniye": data_calback.data.start_milisecond
                    },
                "kilitlenmeBitisZamani":
                    { "saat": data_calback.data.stop_hour,
                    "dakika": data_calback.data.stop_min,
                    "saniye": data_calback.data.stop_second,
                    "milisaniye": data_calback.data.stop_milisecond
                    },
                "otonom_kilitlenme": data_calback.data.otonom}
        url = f'{self.base_url}/kilitlenme_bilgisi'
        response = self.session.post(url, json=data, headers=self.header, timeout=10)
        
        if response.status_code == 200:
            return   sendlockResponse(success = 1,result = response.status_code)
        return   sendlockResponse(success = 0,result = response.status_code)

    def send_qr_callback(self,data_calback):
        

        data = {"kilitlenmeBaslangicZamani":
                    {"saat": data_calback.data.start_hour,
                    "dakika": data_calback.data.start_min,
                    "saniye": data_calback.data.start_second,
                    "milisaniye": data_calback.data.start_milisecond
                    },
                "kilitlenmeBitisZamani":
                    { "saat": data_calback.data.stop_hour,
                    "dakika": data_calback.data.stop_min,
                    "saniye": data_calback.data.stop_second,
                    "milisaniye": data_calback.data.stop_milisecond
                    },
                "qrMetni": data_calback.data.qr_text}
        print(data)
        url = f'{self.base_url}/kamikaze_bilgisi'
        response = self.session.post(url, json=data, headers=self.header, timeout=10)
        
        if response.status_code == 200:
            rospy.loginfo("Qr Text send successfully to server")
            return   sendqrResponse(success = 1,result = response.status_code)
        return   sendqrResponse(success = 0,result = response.status_code)
    
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

    def fcu_time_callback(self, msg):
        self.fcu_time = msg.time_ref.secs
        self.fcu_time_nsecs = msg.time_ref.nsecs

    def qr_check(self, req):
        self.qr_published = False

    def qr_callback(self, msg):
        if (
            self.qr_published is False
        ):  # Check if the QR data has already been published
            self.qr_data = msg.data
            self.publish_qr()
        else:
            rospy.loginfo("qr zaten okundu aga")

    def bbox_callback(self, msg):
        self.bbox = msg
        self.bbox_x = msg.x
        self.bbox_y = msg.y
        self.bbox_w = msg.w
        self.bbox_h = msg.h
        self.process_data()

    def kilit_callback(self, msg):
        self.kilit = msg
        self.process_data()

    def login(self):
        url = f"{self.base_url}/giris"
        headers = {"Content-Type": "application/json",
                   "Accept": "application/json"}
        data = {"kadi": self.username, "sifre": self.password}
        response = self.session.post(
            url, json=data, headers=headers, timeout=10)
        if response.status_code == 200:
            print("Login successful!")
            print(response.json())
            self.team_number = response.json()
        else:
            print(f"Failed to login. Status code: {response.status_code}")
            print("Response:", response.text)
        return response

    def process_data(self):
        if not (
            self.imu
            and self.battery
            and self.rel_alt
            and self.position
            and self.speed
            and self.state
        ):
            return

        if self.state.mode == "AUTO" or self.state.mode.startswith("GUIDED"):
            IHA_otonom = 1  # Set to 1 for autonomous modes
        else:
            IHA_otonom = 0  # Set to 0 for manual modes

        iha_kilitlenme = 1 if self.kilit else 0

        try:
            self.get_server_time()

            # Convert quaternion to euler angles
            roll, pitch, yaw = quaternion_to_euler(
                self.imu.orientation.x,
                self.imu.orientation.y,
                self.imu.orientation.z,
                self.imu.orientation.w,
            )

            # Prepare data dictionary
            data_dict = {
                "takim_numarasi": self.team_number,
                "IHA_enlem": self.position.latitude,
                "IHA_boylam": self.position.longitude,
                "IHA_irtifa": self.rel_alt.data,
                "IHA_yonelme": yaw,
                "IHA_dikilme": pitch,
                "IHA_yatis": roll,
                "IHA_hiz": calculate_speed(
                    self.speed.twist.linear.x,
                    self.speed.twist.linear.y,
                    self.speed.twist.linear.z,
                ),
                "IHA_batarya": int(self.battery.percentage * 100),
                "IHA_otonom": IHA_otonom,
                "iha_kilitlenme": iha_kilitlenme,
                "hedef_merkez_X": self.bbox_x,
                "hedef_merkez_Y": self.bbox_y,
                "hedef_genislik": self.bbox_w,
                "hedef_yukseklik": self.bbox_h,
                "gps_saati": unix_to_utc_formatted(self.fcu_time, self.fcu_time_nsecs),
            }
            logging.info("Prepared data dictionary for server update.")

            # Send data to the server
            response = self.session.post(
                self.server_url_telemetri_gonder, json=data_dict
            )
            logging.info(f"Sent data to server: {data_dict}")

            # Check server response
            if response.status_code == 200:
                logging.info(f"Data sent successfully: {response.json()}")
                self.parse_and_publish_konumBilgileri(response.json())
            else:
                logging.error(
                    f"Failed to send data, status code: {response.status_code}"
                )

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
            logging.error(
                f"An error occurred while parsing and publishing konumBilgileri: {str(e)}"
            )

    def publish_qr(self):
        while not rospy.is_shutdown():
            start_time = rospy.get_param(
                "/kamikazeBaslangicZamani", [0, 0, 0, 0])
            end_time = rospy.get_param("/kamikazeBitisZamani", [0, 0, 0, 0])
            if end_time != [0, 0, 0, 0]:
                break

        if self.qr_data and end_time != [0, 0, 0, 0]:
            try:
                start_time = rospy.get_param(
                    "/kamikazeBaslangicZamani", [0, 0, 0, 0])
                end_time = rospy.get_param(
                    "/kamikazeBitisZamani", [0, 0, 0, 0])

                rospy.loginfo(f"Start Time Retrieved: {start_time}")
                rospy.loginfo(f"End Time Retrieved: {end_time}")

                data_dict = {
                    "kamikazeBaslangicZamani": {
                        "hour": start_time[0],
                        "minute": start_time[1],
                        "second": start_time[2],
                        "millisecond": start_time[3],
                    },
                    "kamikazeBitisZamani": {
                        "hour": end_time[0],
                        "minute": end_time[1],
                        "second": end_time[2],
                        "millisecond": end_time[3],
                    },
                    "qrMetni ": self.qr_data if self.qr_data else "No QR Data",
                }

                print("Correction Timing Dictionary:")
                print(json.dumps(data_dict, indent=4))
                # Here you could send the data, save it to a file, etc.

                response = self.session.post(
                    self.server_url_qr_bilgisi, json=data_dict)
                if response.status_code == 200:
                    logging.info(
                        f"Lock-on data sent successfully: {response.json()}")
                else:
                    logging.error(
                        f"Failed to send lock-on data, status code: {response.status_code}"
                    )

                self.qr_published = True
            except KeyError:
                rospy.logerr(
                    "Failed to retrieve start or end time from ROS parameters."
                )
        else:
            rospy.loginfo("no qr_data available")

    def get_server_time(self):
        try:
            response = self.session.get(
                self.server_url_sunucusaati, timeout=10)
            if response.status_code == 200:
                server_time = response.json()
                logging.info(
                    f"Server time retrieved successfully: {server_time}")
                time_str = f"{server_time['saat']}:{server_time['dakika']}:{server_time['saniye']}:{server_time['milisaniye']}"
                self.server_time_pub.publish(time_str)
                return time_str
            else:
                logging.error(
                    f"Failed to retrieve server time, status code: {response.status_code}"
                )
                return None
        except Exception as e:
            logging.error(
                f"An error occurred while retrieving server time: {str(e)}")
            return None


if __name__ == "__main__":
    rospy.init_node("comm_node", anonymous=True)
    comm_node = Comm_Node()
    comm_node.login()  # perform login
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        logging.info("ROS node interrupted.")
    except Exception as e:
        logging.error(f"An error occurred in the main loop: {str(e)}")