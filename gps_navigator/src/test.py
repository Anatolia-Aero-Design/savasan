import subprocess
import requests
import subprocess
import datetime 

class IhaApiClient:
    def __init__(self, base_url, username, password):
        self.session = requests.Session()
        self.base_url = base_url
        self.username = username
        self.password = password
        self.team_number = None

    def login(self):
        url = f"{self.base_url}/giris"
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        }
        data = {
            "kadi": self.username,
            "sifre": self.password
        }
        response = self.session.post(url, json=data, headers=headers, timeout=10)
        if response.status_code == 200:
            print("Login successful!")
            print(response.json())
            self.team_number = response.json()
        else:
            print(f"Failed to login. Status code: {response.status_code}")
            print("Response:", response.text)
        return response

    def get_server_time(self):
        url = f"{self.base_url}/sunucusaati"
        response = self.session.get(url, timeout=10)
        if response.status_code == 200:
            print("Server time retrieved successfully!")
            print("Server Time:", response.json())
        else:
            print(f"Failed to retrieve server time. Status code: {response.status_code}")
            print("Response:", response.text)
        return response

    def send_telemetry(self, telemetry_data):
        url = f"{self.base_url}/telemetri_gonder"
        response = self.session.post(url, json=telemetry_data, timeout=10)
        if response.status_code == 200:
            print("Telemetry data sent successfully!")
            print("Response:", response.json())
        else:
            print(f"Failed to send telemetry data. Status code: {response.status_code}")
            print("Response:", response.text)
        return response

    def send_lock_info(self, lock_info):
        url = f"{self.base_url}/kilitlenme_bilgisi"
        response = self.session.post(url, json=lock_info, timeout=10)
        if response.status_code == 200:
            print("Lock information sent successfully!")
        else:
            print(f"Failed to send lock information. Status code: {response.status_code}")
            print("Response:", response.text)
        return response

    def send_kamikaze_info(self, kamikaze_info):
        url = f"{self.base_url}/kamikaze_bilgisi"
        response = self.session.post(url, json=kamikaze_info, timeout=10)
        if response.status_code == 200:
            print("Kamikaze information sent successfully!")
        else:
            print(f"Failed to send kamikaze information. Status code: {response.status_code}")
            print("Response:", response.text)
        return response

    def get_qr_coordinates(self):
        url = f"{self.base_url}/qr_koordinati"
        response = self.session.get(url, timeout=10)
        if response.status_code == 200:
            print("QR coordinates retrieved successfully!")
            print("QR Coordinates:", response.json())
        else:
            print(f"Failed to retrieve QR coordinates. Status code: {response.status_code}")
            print("Response:", response.text)
        return response

    def get_hss_coordinates(self):
        url = f"{self.base_url}/hss_koordinatlari"
        response = self.session.get(url, timeout=10)
        if response.status_code == 200:
            print("HSS coordinates retrieved successfully!")
            print("HSS Coordinates:", response.json())
        else:
            print(f"Failed to retrieve HSS coordinates. Status code: {response.status_code}")
            print("Response:", response.text)
        return response

def set_system_time(self, date_time_str):
        try:
            # The `date_time_str` should be in the format 'YYYY-MM-DD HH:MM:SS'
            subprocess.run(
                ['sudo', 'date', '--set', date_time_str], check=True)
            print(f"System time set to {date_time_str}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to set system time: {e}")

# Example Usage
if __name__ == "__main__":
    base_url = 'http://10.0.0.10:10001/api'   
    username = 'estuanatolia'
    password = '2Eqtm3v3ZJ'

    # Initialize the API client
    client = IhaApiClient(base_url, username, password)

    # Perform login
    client.login()

    # Get server time
    sunucusaati = client.get_server_time()


    # Example JSON response
    json_time = sunucusaati.json()

    # Get the current date (we assume the current year and month)

    # For Linux (uses `date` command)
    def set_system_time(date_time_str):
        try:
            # The `date_time_str` should be in the format 'YYYY-MM-DD HH:MM:SS'
            subprocess.run(['sudo', 'date', '--set', date_time_str], check=True)
            print(f"System time set to {date_time_str}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to set system time: {e}")
    current_date = datetime.datetime.now()

    # Create the datetime string in the format 'YYYY-MM-DD HH:MM:SS'
    date_time_str = f"{current_date.year}-{current_date.month:02d}-{json_time['gun']:02d} " \
                    f"{json_time['saat']:02d}:{json_time['dakika']:02d}:{json_time['saniye']:02d}"

    # Set the system time
    set_system_time(date_time_str)
    

    

