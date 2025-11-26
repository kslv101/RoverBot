# app.py
import os
import json
import yaml
import socket
from flask import Flask, render_template, send_file, request, jsonify

# Конвертация PGM → PNG (требуется Pillow)
try:
    from PIL import Image
except ImportError:
    print("Установите Pillow: pip install Pillow")
    exit(1)

app = Flask(__name__)

# Настройки по умолчанию
ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 5005
MAP_NAME = "warehouse"  # без расширения

@app.route('/')
def index():
    return render_template('index.html', robot_ip=ROBOT_IP, map_name=MAP_NAME)

@app.route('/set_robot_ip', methods=['POST'])
def set_robot_ip():
    global ROBOT_IP
    data = request.json
    ROBOT_IP = data.get('ip', '127.0.0.1')
    return jsonify({"status": "ok", "ip": ROBOT_IP})

@app.route('/map_image')
def map_image():

    png_path = f"maps/{MAP_NAME}.png"
    
    if not os.path.exists(png_path):
        return "Карта не найдена", 404
    
    return send_file(png_path, mimetype='image/png')

@app.route('/targets')
def get_targets():
    """Возвращает список целей из YAML"""
    yaml_path = f"maps/{MAP_NAME}_targets.yaml"
    if not os.path.exists(yaml_path):
        return jsonify([]), 404
    
    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return jsonify(data.get('targets', []))

@app.route('/send_command', methods=['POST'])
def send_command():
    try:
        data = request.json
        cmd = data.get('cmd')
        
        if cmd == "select_target":
            payload = {"cmd": "select_target", "id": data['id']}
        elif cmd == "set_initial_pose":
            payload = {"cmd": "set_initial_pose", "x": data['x'], "y": data['y']}
        elif cmd == "plan_path":
            payload = {"cmd": "plan_path", "target_id": data['target_id']}
        elif cmd in ["start", "stop"]:
            payload = {"cmd": cmd}
        else:
            return jsonify({"error": "Unknown command"}), 400

        # Отправка по UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(json.dumps(payload).encode('utf-8'), (ROBOT_IP, ROBOT_PORT))
        sock.close()
        
        # Если это plan_path — имитируем ответ с путём (для демо)
        # В реальности робот должен отправить путь на другой порт GUI
        if cmd == "plan_path":
            # Пример пути (замените на реальный при интеграции)
            fake_path = [
                {"x": 1.0, "y": 1.0},
                {"x": 2.0, "y": 2.0},
                {"x": 3.0, "y": 3.0}
            ]
            return jsonify({"status": "path_planned", "path": fake_path})
        
        return jsonify({"status": "sent", "command": payload})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)