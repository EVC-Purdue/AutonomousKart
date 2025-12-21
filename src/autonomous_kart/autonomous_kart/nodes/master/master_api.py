import threading

import rclpy
from flask import Flask, jsonify, request

from .master_node import MasterNode

app = Flask(__name__)
master_node: MasterNode | None = None

@app.route("/", methods=['GET'])
def ping():
    return jsonify({'ping': 'pong'})

@app.route("/logs", methods=["GET"])
def logs():
    if not master_node:
        return jsonify({"error": "master node not initialized"}), 500
    return jsonify(master_node.get_logs())

@app.route("/manual_control", methods=["POST"])
def logs():
    data = request.get_json()
    if not isinstance(data, dict):
        return jsonify({"error": "invalid or missing JSON body"}), 400

    if "speed" not in data or "steering" not in data:
        return jsonify({"error": "missing 'speed' or 'steering' field"}), 400

    speed, steering = float(data['speed']), float(data['steering'])
    master_node.manual_control(speed, steering)
    return jsonify({"success": 200})

@app.route("/set_state", methods=["POST"])
def set_state():
    data = request.get_json()
    if not isinstance(data, dict):
        return jsonify({"error": "invalid or missing JSON body"}), 400

    if "state" not in data:
        return jsonify({"error": "state field not present"})
    state = data['state']
    if state not in ["IDLE", "MANUAL", "AUTONOMOUS", "STOPPED"]:
        return jsonify({"error": f"state {state} is not a valid state."})

    master_node.update_state(state)
    return jsonify({"success": 200})

def start(node: MasterNode) -> None:
    rclpy.spin(node)


def main():
    global master_node

    rclpy.init()
    master_node = MasterNode()
    ros_thread = threading.Thread(target=start, args=(master_node,), daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=8000, debug=False)

    rclpy.shutdown()

if __name__ == "__main__":
    main()