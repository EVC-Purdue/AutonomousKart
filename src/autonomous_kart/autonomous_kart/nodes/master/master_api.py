import threading
import os, csv

import rclpy
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS

from .master_node import MasterNode, STATES

app = Flask(__name__)
CORS(app)
master_node: MasterNode | None = None

@app.after_request
def cors(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    return response

@app.route("/", methods=["GET"])
def ping():
    return jsonify({"ping": "pong"})


@app.route("/get_logs", methods=["GET"])
def get_logs():
    if not master_node:
        return jsonify({"error": "master node not initialized"}), 500
    return jsonify(master_node.get_logs())


@app.route("/manual_control", methods=["POST"])
def manual_control():
    data = request.get_json()
    if not isinstance(data, dict):
        return jsonify({"error": "invalid or missing JSON body"}), 400

    if "speed" not in data or "steering" not in data:
        return jsonify({"error": "missing 'speed' or 'steering' field"}), 400

    speed, steering = float(data["speed"]), float(data["steering"])
    master_node.manual_control(speed, steering)
    return jsonify({"success": "ok"})


@app.route("/set_state", methods=["POST"])
def set_state():
    data = request.get_json()
    if not isinstance(data, dict):
        return jsonify({"error": "invalid or missing JSON body"}), 400

    if "state" not in data:
        return jsonify({"error": "state field not present"})
    state = data["state"]
    if state not in [s.value for s in STATES]:
        return jsonify({"error": f"state {state} is not a valid state."})

    master_node.update_state(state)
    return jsonify({"success": "ok"})

@app.route("/get_state", methods=["GET"])
def get_state():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify({"state": master_node.state})

@app.route("/odom", methods=["GET"])
def odom():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_odom())


@app.route("/racing_line", methods=["GET"])
def racing_line():
    path = "/ws/data/racing_line/line.csv"
    if not os.path.exists(path):
        return jsonify({"points": []})
    points = []
    with open(path, "r") as f:
        for row in f:
            parts = row.strip().split(",")
            if len(parts) >= 3:
                try:
                    points.append([float(parts[1]), float(parts[2])])
                except ValueError:
                    continue
    return jsonify({"points": points})


@app.route("/viz")
def viz():
    return send_from_directory("/ws/viz", "viz.html")


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
