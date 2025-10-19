# -*- coding: utf-8 -*-
from flask import Flask, render_template, request, jsonify
import socket
import json

app = Flask(__name__)

# Настройки по умолчанию
ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 5005

@app.route('/')
def index():
    return render_template('index.html', robot_ip=ROBOT_IP)

@app.route('/set_robot_ip', methods=['POST'])
def set_robot_ip():
    global ROBOT_IP
    data = request.json
    ROBOT_IP = data['ip']
    return jsonify({"status": "ok"})

@app.route('/send_command', methods=['POST'])
def send_command():
    try:
        data = request.json
        cmd = data['cmd']
        
        # Формируем JSON-команду
        if cmd == "set_target":
            payload = {"cmd": "set_target", "type": data['target_type']}
        elif cmd in ["start", "stop"]:
            payload = {"cmd": cmd}
        else:
            return jsonify({"error": "Unknown command"}), 400

        # Отправляем по UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(json.dumps(payload).encode(), (ROBOT_IP, ROBOT_PORT))
        sock.close()
        
        return jsonify({"status": "sent", "command": payload})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)