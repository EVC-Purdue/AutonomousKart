import threading

import rclpy
from flask import Flask, jsonify

from .metrics_node import MetricsNode

app = Flask(__name__)
metrics_node: MetricsNode | None = None

@app.route("/", methods=['GET'])
def ping():
    return jsonify({'ping': 'pong'})


@app.route("/command_rate", methods=["GET"])
def command_rate():
    if not metrics_node:
        return jsonify({"error": "metrics node not initialized"}), 500
    return jsonify(metrics_node.last_avg_rate)


@app.route("/logs", methods=["GET"])
def logs():
    if not metrics_node:
        return jsonify({"error": "metrics node not initialized"}), 500
    return jsonify(metrics_node.get_logs())


def start(node: MetricsNode) -> None:
    rclpy.spin(node)


def main():
    global metrics_node

    rclpy.init()
    metrics_node = MetricsNode()
    ros_thread = threading.Thread(target=start, args=(metrics_node,), daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=8000, debug=False)

    rclpy.shutdown()

if __name__ == "__main__":
    main()