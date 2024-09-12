#!/usr/bin/env python

from unittest import result
import rospy
from sensor_msgs.msg import Imu, BatteryState, NavSatFix, TimeReference
from std_msgs.msg import Float64, Bool, String
from std_srvs.srv import Trigger
from server_comm.srv import sendlock, sendlockResponse, sendqr, sendqrResponse, gethss, gethssResponse


from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
import requests
from server_comm.msg import KonumBilgileri, KonumBilgisi, Kilitlenme, HavaSavunmaKoordinatlari, HavaSavunmaKoordinati

from utils import quaternion_to_euler, calculate_speed, unix_to_utc_formatted

from image_processor.msg import Yolo_xywh
import threading


class Comm_Node:
    def __init__(self):
        self.session = requests.Session()
        self.base_url = "http://127.0.0.1:5000/api"
        self.username = "estuanatolia"
        self.password = "2Eqtm3v3ZJ"
        self.team_number = 69
        self.header = {"Content-Type": "application/json",
                       "Accept": "application/json"}

        # Get server APIs from sim.launch params
        self.server_url_telemetri_gonder = rospy.get_param(
            "/comm_node/api/telemetri_gonder"
        )
        self.server_url_sunucusaati = rospy.get_param(
            "/comm_node/api/sunucusaati")
        self.server_url_kamikaze_bilgisi = rospy.get_param(
            "/comm_node/api/kamikaze_bilgisi"
        )

        self.get_hss_coordinate_message = rospy.Service(
            "get_hss_coordinates", gethss, self.get_hss_coordinates)

        self.send_lock_message = rospy.Service(
            "send_lock_message", sendlock, self.send_lock_callback)
        self.send_qr_message = rospy.Service(
            "send_qr_message", sendqr, self.send_qr_callback)

        self.imu = None
        self.battery = None
        self.rel_alt = None
        self.position = None
        self.speed = None
        self.state = None
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
            self.fcu_time_sub = rospy.Subscriber(
                "/mavros/time_reference", TimeReference, self.fcu_time_callback
            )
            self.kilit_sub = rospy.Subscriber(
                "/kilit", Bool, self.kilit_callback)

        except Exception as e:
            print("error")
            rospy.logerr(f"An error occurred in process_data: {str(e)}")

        # Initialize Publishers
        self.server_time_pub = rospy.Publisher(
            "/server_time", String, queue_size=10)
        self.konum_pub = rospy.Publisher(
            "/konum_bilgileri", KonumBilgileri, queue_size=10
        )
        self.hss_pub = rospy.Publisher(
            "/hss_locations", HavaSavunmaKoordinatlari, queue_size=10)

        # Create a thread for sending telemetry data
        self.telemetry_thread = threading.Thread(target=self.send_telem_loop)
        # Ensure the thread will exit when the main program ends
        self.telemetry_thread.daemon = True

    def start_telem_loop(self):
        self.telemetry_thread.start()

    def send_telem_loop(self):
        rate = rospy.Rate(1)  # 10 Hz loop
        while not rospy.is_shutdown():
            self.sent_telem()
            rate.sleep()

    def get_hss_coordinates(self, _):
        try:
            url = f'{self.base_url}/hss_koordinatlari'
            response = self.session.get(url)
            data_HavaSavunmaKoordinatlari = HavaSavunmaKoordinatlari()

            if response.status_code == 200:
                response_data = response.json().get('hss_koordinat_bilgileri')
                for data in response_data:
                    data_HavaSavunmaKoordinati = HavaSavunmaKoordinati()
                    data_HavaSavunmaKoordinati.hssBoylam = data["hssBoylam"]
                    data_HavaSavunmaKoordinati.hssYaricap = data["hssYaricap"]
                    data_HavaSavunmaKoordinati.hssEnlem = data["hssEnlem"]
                    data_HavaSavunmaKoordinati.id = data["id"]
                    data_HavaSavunmaKoordinatlari.hss_Koordinati.append(
                        data_HavaSavunmaKoordinati)

                rospy.loginfo(
                    f"Air defense coordinates retrieved successfully")
                self.hss_pub.publish(data_HavaSavunmaKoordinatlari)
                return gethssResponse(success=1, result=response.status_code, data=data_HavaSavunmaKoordinatlari)
            else:
                rospy.logerr(
                    f"Failed to retrieve air defense coordinates, status code: {response.status_code}")
                return gethssResponse(success=0, result=response.status_code)
        except Exception as e:
            rospy.logerr(
                f"An error occurred while retrieving air defense coordinates: {str(e)}")
            return gethssResponse(success=0, result=response.status_code)

    def parse_and_publish_hss_coordinates_callback(self, data_callback):
        try:
            response_json = self.get_hss_coordinates()
            hss_koordinatlari_msg = HavaSavunmaKoordinatlari()
            hss_coordinate_data = response_json.get(
                "HavaSavunmaKoordinatlari", [])

            for item in hss_coordinate_data:
                hss_coordinate = HavaSavunmaKoordinati()
                hss_coordinate.id = item["id"]
                hss_coordinate.hssEnlem = item["hssEnlem"]
                hss_coordinate.hssBoylam = item["hssBoylam"]
                hss_coordinate.hssYaricap = item["hssYaricap"]

                hss_koordinatlari_msg.hss_Koordinati.append(hss_coordinate)

        except Exception as e:
            rospy.logerr(
                f"An error occurred while parsing and publishing hss_Koordinati: {str(e)}"
            )

    def send_lock_callback(self, data_calback):

        data = {"kilitlenmeBaslangicZamani":
                {"saat": data_calback.data.start_hour,
                 "dakika": data_calback.data.start_min,
                 "saniye": data_calback.data.start_second,
                 "milisaniye": data_calback.data.start_milisecond
                 },
                "kilitlenmeBitisZamani":
                    {"saat": data_calback.data.stop_hour,
                     "dakika": data_calback.data.stop_min,
                     "saniye": data_calback.data.stop_second,
                     "milisaniye": data_calback.data.stop_milisecond
                     },
                "otonom_kilitlenme": data_calback.data.otonom}
        url = f'{self.base_url}/kilitlenme_bilgisi'
        response = self.session.post(
            url, json=data, headers=self.header, timeout=10)

        if response.status_code == 200:
            return sendlockResponse(success=1, result=response.status_code)
        return sendlockResponse(success=0, result=response.status_code)

    def send_qr_callback(self, data_calback):

        data = {"kamikazeBaslangicZamani":
                {"saat": data_calback.data.start_hour,
                 "dakika": data_calback.data.start_min,
                 "saniye": data_calback.data.start_second,
                 "milisaniye": data_calback.data.start_milisecond
                 },
                "kamikazeBitisZamani":
                    {"saat": data_calback.data.stop_hour,
                     "dakika": data_calback.data.stop_min,
                     "saniye": data_calback.data.stop_second,
                     "milisaniye": data_calback.data.stop_milisecond
                     },
                "qrMetni": data_calback.data.qr_text}
        url = f'{self.base_url}/kamikaze_bilgisi'
        response = self.session.post(
            url, json=data, headers=self.header, timeout=10)

        if response.status_code == 200:
            rospy.loginfo("Qr Text send successfully to server")
            return sendqrResponse(success=1, result=response.status_code)
        return sendqrResponse(success=0, result=response.status_code)

    def imu_callback(self, msg):
        self.imu = msg

    def battery_callback(self, msg):
        self.battery = msg

    def rel_altitude_callback(self, msg):
        self.rel_alt = msg

    def position_callback(self, msg):
        self.position = msg

    def speed_callback(self, msg):
        self.speed = msg

    def state_callback(self, msg):
        self.state = msg

    def fcu_time_callback(self, msg):
        self.fcu_time = msg.time_ref.secs
        self.fcu_time_nsecs = msg.time_ref.nsecs

    def bbox_callback(self, msg):
        self.bbox = msg
        self.bbox_x = msg.x
        self.bbox_y = msg.y
        self.bbox_w = msg.w
        self.bbox_h = msg.h

    def kilit_callback(self, msg):
        if msg:
            self.kilit = 1
        else:
            self.kilit = 0

    def login(self):
        url = f"{self.base_url}/giris"
        headers = {"Content-Type": "application/json",
                   "Accept": "application/json"}
        data = {"kadi": self.username, "sifre": self.password}
        response = self.session.post(
            url, json=data, headers=headers, timeout=10)
        if response.status_code == 200:
            rospy.logwarn("Login successful!")
            rospy.loginfo(response.json())
            self.team_number = response.json()
        else:
            rospy.logerr(f"Failed to login. Status code: {response.status_code}")
            rospy.logerr("Response:", response.text)
        return response

    def sent_telem(self):

        if not (
            self.imu
            and self.battery
            and self.rel_alt
            and self.position
            and self.speed
            and self.state
        ):
            pass
        try:

            if self.state.mode == "MANUAL" or self.state.mode == "FBWA":
                IHA_otonom = 0
            else:
                IHA_otonom = 1
        except:

            rospy.logerr(f"mode secerken hata{self.state}")
            IHA_otonom = 0
        try:
            if self.imu is not None and hasattr(self.imu, 'orientation'):
                roll, pitch, yaw = quaternion_to_euler(
                    self.imu.orientation.x,
                    self.imu.orientation.y,
                    self.imu.orientation.z,
                    self.imu.orientation.w,
                )
            else:
                rospy.logwarn("IMU data is not available or incomplete.")
                return  # or set default values for roll, pitch, yaw
        except Exception as e:
            rospy.logerr(f"Error processing IMU data: {e}")

            # Prepare data dictionary
            data_dict = {
                "takim_numarasi": self.team_number,
                "iha_enlem": self.position.latitude,
                "iha_boylam": self.position.longitude,
                "iha_irtifa": self.rel_alt.data,
                "iha_dikilme": pitch,
                "iha_yonelme": yaw,
                "iha_yatis": roll,
                "iha_hizi": calculate_speed(
                    self.speed.twist.linear.x,
                    self.speed.twist.linear.y,
                ),
                "iha_batarya": int(self.battery.percentage * 100),
                "iha_otonom": IHA_otonom,
                "iha_kilitlenme": int(self.kilit),
                "iha_kilitlenme": int(self.kilit),
                "hedef_merkez_X": self.bbox_x,
                "hedef_merkez_Y": self.bbox_y,
                "hedef_genislik": self.bbox_w,
                "hedef_yukseklik": self.bbox_h,
                "gps_saati": unix_to_utc_formatted(self.fcu_time, self.fcu_time_nsecs)
            }

            # Send data to the server
            response = self.session.post(
                self.server_url_telemetri_gonder, json=data_dict
            )

            # Check server response
            if response.status_code == 200:
                self.parse_and_publish_konumBilgileri(response.json())
            else:
                rospy.logerr(
                    f"Failed to send data, status code: {response}"
                )

        except Exception as e:
            rospy.logerr(f"An error occurred in process_data: {e}")

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
            rospy.loginfo("Published konumBilgileri message.")

        except Exception as e:
            rospy.logerr(
                f"An error occurred while parsing and publishing konumBilgileri: {str(e)}"
            )


if __name__ == "__main__":
    rospy.init_node("comm_node", anonymous=True)
    comm_node = Comm_Node()
    # comm_node.login()  # perform login
    comm_node.start_telem_loop()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred in the main loop: {str(e)}")
