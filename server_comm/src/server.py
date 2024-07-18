from flask import Flask, request, jsonify
from Competition import Competition
import logging

app = Flask(__name__)

@app.route('/api/telemetri_gonder', methods=['POST'])
def update_data():
    json_data = request.get_json()
    competition.update_contestant(json_data['takim_numarasi'], json_data)
    response = competition.response_json()

    # Print the received data
    print(f"Received data: {json_data}")
    return jsonify(response), 200

@app.route('/api/kilitlenme_bilgisi', methods=['POST'])
def update_lock_on():
    json_data = request.get_json()
    print(f"Received lock-on data: {json_data}")
    competition.update_lock_on(json_data)
    response = competition.response_json()

    return jsonify({"status": "success"}), 200

@app.route('/api/sunucusaati', methods=['GET'])
def get_server_time():
    current_time = competition.get_current_time()  # Add a method to get only the current time
    return jsonify({"sunucusaati": current_time}), 200
    


if __name__ == '__main__':
    competition = Competition()
    print("Initialized competition object:", competition)
    app.run(debug=True, host='127.0.0.1', port=5000)
