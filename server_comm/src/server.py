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
                "hssEnlem": 36.939879,
                "hssBoylam": 35.532268,
                "hssYaricap": 50
            },
            {
                "id": 1,
                "hssEnlem": 36.940047,
                "hssBoylam": 35.537481,
                "hssYaricap": 50
            },
            {
                "id": 2,
                "hssEnlem": 36.936330,
                "hssBoylam": 35.531538,
                "hssYaricap": 75
            },
            {
                "id": 3,
                "hssEnlem": 36.937116,
                "hssBoylam": 35.537972,
                "hssYaricap": 150
            }
        ]
    }
    return jsonify(hss_koordinatlari), 200

@app.route('/api/qr_koordinati', methods=['GET'])
def get_qr_coordinates():
    qr_latitude   = 36.938828
    qr_longtitude = 35.532133
    qr_coordinates = {
        "qrEnlem": qr_latitude,
        "qrBoylam": qr_longtitude
    }
    return jsonify(qr_coordinates), 200
    
@app.route('/api/kilitlenme_bilgisi', methods=['POST'])
def kenetlenme_bilgisi_gonder():
    json_data = request.get_json()
    print(f"Received lock-on data: {json_data}")
    print(json_data)
    competition.update_lock_on(json_data)
    response = competition.response_json()

    return jsonify({"status": "success"}), 200
               
if __name__ == '__main__':
    competition = Competition()
    app.run(debug=True, host='127.0.0.1', port=5000)