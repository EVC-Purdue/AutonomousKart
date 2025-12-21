import threading

import rclpy
from flask import Flask, jsonify

from .master_node import MasterNode

app = Flask(__name__)
metrics_node: MasterNode | None = None

@app.route("/", methods=['GET'])
def ping():
    return jsonify({'ping': 'pong'})

@app.route("/logs", methods=["GET"])
def logs():
    if not metrics_node:
        return jsonify({"error": "metrics node not initialized"}), 500
    return jsonify(metrics_node.get_logs())


def start(node: MasterNode) -> None:
    rclpy.spin(node)


def main():
    global metrics_node

    rclpy.init()
    metrics_node = MasterNode()
    ros_thread = threading.Thread(target=start, args=(metrics_node,), daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=8000, debug=False)

    rclpy.shutdown()

if __name__ == "__main__":
    main()