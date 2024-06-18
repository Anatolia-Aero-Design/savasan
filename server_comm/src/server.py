from flask import Flask, request, jsonify
from Competition import Competition
import logging

app = Flask(__name__)

@app.route('/update_data', methods=['POST'])
def update_data():
    json_data = request.get_json()
    competition.update_contestant(request.get_json('takim_numarasi') , json_data)
    response = competition.response_json()
    
    return jsonify(response), 200
    


if __name__ == '__main__':
    competition = Competition()
    app.run(debug=True, host='172.31.32.149', port=5000)