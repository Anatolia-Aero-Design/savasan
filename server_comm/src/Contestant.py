import time
from typing import Any
class Contestant():
    def __init__(self):
        pass
        
   
    def update(self,json_data):
        self.id = json_data['takim_numarasi']
        self.lat = json_data['IHA_enlem']
        self.lon = json_data['IHA_boylam']
        self.altitude = json_data['IHA_irtifa']
        self.pitch = json_data['IHA_dikilme']
        self.roll = json_data['IHA_yonelme']
        self.yaw = json_data['IHA_yatis']
        self.speed = json_data['IHA_hiz']
        self.battery = json_data['IHA_batarya']
        self.otonom = json_data['IHA_otonom']
        self.kilitlenme = json_data['iha_kilitlenme']
        self.merkez_X = json_data['hedef_merkez_X']
        self.merkez_Y = json_data['hedef_merkez_Y']
        self.genislik = json_data['hedef_genislik']
        self.yukseklik = json_data['hedef_yukseklik']
        time.sleep(1)
        
    def get_info(self):
        response = {
            "takim_numarasi" : self.id , 
            "IHA_enlem" : self.lat ,
            "IHA_boylam" : self.lon ,
            "IHA_irtifa" : self.altitude ,
            "IHA_dikilme" : self.pitch ,
            "IHA_yonelme" : self.yaw ,
            "IHA_yatis" : self.roll ,
            "IHA_hiz" : self.speed,
            "IHA_zamanfarki" : 0 ,
        }
        return response
    
    
