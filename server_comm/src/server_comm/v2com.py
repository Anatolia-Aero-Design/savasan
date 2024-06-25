#!/usr/bin/env python

from dronekit import connect, VehicleMode
import requests
import logging
import time

# Import utility functions (ensure these are defined somewhere)
from utils import quaternion_to_euler, calculate_speed, mode_guided

# Configure logging
logging.basicConfig(filename='/home/valvarn/catkin_ws/logs/serverlog.log', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def connect_to_vehicle(connection_string):
    try:
        vehicle = connect(connection_string, wait_ready=True)
        logging.info("Connected to vehicle")
        return vehicle
    except Exception as e:
        print(f"Error connecting to vehicle: {str(e)}")
        return None

def get_vehicle_data(vehicle):
    try:
        # Get IMU data
        imu_data = vehicle.attitude

        # Convert quaternion to euler angles
        roll, pitch, yaw = imu_data.roll, imu_data.pitch, imu_data.yaw

        # Prepare data dictionary
        data_dict = {
            "takim_numarasi": 1,
            "IHA_enlem": vehicle.location.global_frame.lat,
            "IHA_boylam": vehicle.location.global_frame.lon,
            "IHA_irtifa": vehicle.location.global_relative_frame.alt,
            "IHA_yonelme": yaw,
            "IHA_dikilme": pitch,
            "IHA_yatis": roll,
            "IHA_hiz": calculate_speed(vehicle.velocity[0], vehicle.velocity[1], vehicle.velocity[2]),
            "IHA_batarya": vehicle.battery.level,
            "IHA_otonom": mode_guided(vehicle.mode.name == 'GUIDED')
        }

        return data_dict

    except Exception as e:
        print(f"Error getting vehicle data: {str(e)}")
        return None

def send_data_to_server(data_dict):
    try:
        # Server URL
        server_url = 'http://172.31.32.149:5000/update_data'

        # Send data to the server
        response = requests.post(server_url, json=data_dict)

        # Check server response
        if response.status_code == 200:
            print(f"Data sent successfully: {response.json()}")
        else:
            print(f"Failed to send data, status code: {response.status_code}")

    except Exception as e:
        print(f"An error occurred while sending data to the server: {str(e)}")

if __name__ == '__main__':
    try:
        print('hello')
        # Connection string to the vehicle
        connection_string = '127.0.0.1:14570'  # Change this to your connection string

        # Connect to vehicle
        vehicle = connect_to_vehicle(connection_string)

        if vehicle:
            while True:
                # Get vehicle data
                data_dict = get_vehicle_data(vehicle)

                if data_dict:
                    # Send data to server
                    send_data_to_server(data_dict)

                # Sleep for a while before sending the next data packet
                time.sleep(1)
        else:
            print("Failed to connect to vehicle")

    except Exception as e:
        print(f"An error occurred in the main loop: {str(e)}")
