import json
from Contestant import Contestant
from datetime import datetime

class Competition():
    def __init__(self) -> None:
        self.contestants = []
        
    
    def find_contestant(self,id):
        for index, a in enumerate(self.contestants):
            if a.id == id:
                return index
        return -1
    
    def update_contestant(self,id,json_data):
        id = json_data["takim_numarasi"]
        contestant_index = self.find_contestant(id)
        if contestant_index < 0:
            contestant = Contestant()
            contestant.update(json_data)
            self.contestants.append(contestant)
        else:
            self.contestants[contestant_index].update(json_data)
        
    
    def response_json(self):
        current_time = datetime.now()
        response_json = {
            "sunucusaati": {
            "gun": current_time.day,
            "saat": current_time.hour,
            "dakika": current_time.minute,
            "saniye": current_time.second,
            "milisaniye": current_time.microsecond // 1000 
        },
            "konumBilgileri": [
                
            ]}

        for contestant in self.contestants:
            response_json["konumBilgileri"].append(contestant.get_info())
        
        return response_json