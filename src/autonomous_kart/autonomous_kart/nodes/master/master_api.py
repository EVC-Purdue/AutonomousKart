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
    speed, steering = data['speed'], data['steering']
    master_node.manual_control(speed, steering)
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