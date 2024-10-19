import time
from typing import Any
class Contestant():
    def __init__(self):
        pass
        
   
    def update(self,json_data):
        self.id = json_data['takim_numarasi']
        self.lat = json_data['iha_enlem']
        self.lon = json_data['iha_boylam']
        self.altitude = json_data['iha_irtifa']
        self.pitch = json_data['iha_dikilme']
        self.roll = json_data['iha_yonelme']
        self.yaw = json_data['iha_yatis']
        self.speed = json_data['iha_hiz']
        self.battery = json_data['iha_batarya']
        self.otonom = json_data['iha_otonom']
        self.kilitlenme = json_data['iha_kilitlenme']
        self.merkez_X = json_data['hedef_merkez_X']
        self.merkez_Y = json_data['hedef_merkez_Y']
        self.genislik = json_data['hedef_genislik']
        self.yukseklik = json_data['hedef_yukseklik']
        time.sleep(1)
        
    def get_info(self):
        print(self.id)
        response = {
            "takim_numarasi" : self.id , 
            "iha_enlem" : self.lat ,
            "iha_boylam" : self.lon ,
            "iha_irtifa" : self.altitude ,
            "iha_dikilme" : self.pitch ,
            "iha_yonelme" : self.yaw ,
            "iha_yatis" : self.roll ,
            "iha_hizi" : self.speed,
            "zaman_farki" : 0 ,
        }
        return response
    
    
