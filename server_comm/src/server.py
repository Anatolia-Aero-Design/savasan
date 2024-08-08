from flask import Flask, request, jsonify
from Competition import Competition
import logging

app = Flask(__name__)

@app.route('/api/giris', methods=['POST'])
def login():
    ...

@app.route('/api/telemetri_gonder', methods=['POST'])
def update_data():
    json_data = request.get_json()
    competition.update_contestant(request.get_json('takim_numarasi') , json_data)
    response = competition.response_json()
    
    return jsonify(response), 200

@app.route('/api/sunucusaati', methods=['GET'])
def get_server_time():
    current_time = competition.get_current_time()  # Add a method to get only the current time
    return jsonify({"sunucusaati": current_time}), 200

@app.route('/api/hss_koordinatlari', methods=['GET'])
def hss_coordinates():
    hss_koordinatlari = {
        "sunucusaati": competition.get_current_time(),
        "hss_koordinat_bilgileri": [
            {
                "id": 0,
                "hssEnlem": -35.360947,
                "hssBoylam": 149.164313,
                "hssYaricap": 50
            },
            {
                "id": 1,
                "hssEnlem": -35.362358,
                "hssBoylam": 149.167855,
                "hssYaricap": 50
            },
            {
                "id": 2,
                "hssEnlem": -35.364373,
                "hssBoylam": 149.163976,
                "hssYaricap": 75
            },
            {
                "id": 3,
                "hssEnlem": -35.362722,
                "hssBoylam": 149.160419,
                "hssYaricap": 150
            }
        ]
    }
    return jsonify(hss_koordinatlari), 200

@app.route('/api/qr_koordinati', methods=['GET'])
def get_qr_coordinates():
    qr_latitude = -35.360849
    qr_longtitude = 149.161835
    qr_coordinates = {
        "qrEnlem": qr_latitude,
        "qrBoylam": qr_longtitude
    }
    return jsonify(qr_coordinates), 200
    
@app.route('/api/kilitlenme_bilgisi', methods=['POST'])
def kenetlenme_bilgisi_gonder():
    ...    

@app.route('/api/kamikaze_bilgisi', methods=['POST'])
def kamikaze_bilgisi_gonder():
    ...
               
if __name__ == '__main__':
    competition = Competition()
    app.run(debug=True, host='127.0.0.1', port=5000)