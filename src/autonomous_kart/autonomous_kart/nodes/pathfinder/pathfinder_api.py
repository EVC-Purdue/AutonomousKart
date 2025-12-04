import threading

import rclpy
from flask import Flask, jsonify, request

from .pathfinder_node import PathfinderNode

app = Flask(__name__)
metrics_node: PathfinderNode | None = None


@app.route("/", methods=["GET"])
def ping():
    return jsonify({"ping": "pong"})


@app.route("/go_forward", methods=["GET"])
def go_forward():
    pass


@app.route("/go_backwards", methods=["GET"])
def go_backwards():
    pass


@app.route("/turn_left", methods=["GET"])
def turn_left():
    pass


@app.route("/turn_right", methods=["GET"])
def turn_right():
    pass


@app.route("/run_test", methods=["POST"])
def run_test():
    data = request.get_json()
    testIndex, steering, speed = data["test"], data["steering"], data["speed"]
    if testIndex not in {1, 2, 3}:
        return jsonify({"error": "Index should be 1, 2, or 3"}, 400)

    if testIndex == 1:
        pass
    elif testIndex == 2:
        pass
    else:
        pass


@app.route("/logs", methods=["GET"])
def logs():
    if not metrics_node:
        return jsonify({"error": "metrics node not initialized"}), 500
    return jsonify(metrics_node.get_logs())


def start(node: PathfinderNode) -> None:
    rclpy.spin(node)


def main():
    global pathfinder_node

    rclpy.init()
    pathfinder_node = PathfinderNode()
    ros_thread = threading.Thread(target=start, args=(metrics_node,), daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=8001, debug=False)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
