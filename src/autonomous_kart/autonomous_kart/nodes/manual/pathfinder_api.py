import threading
import time

import rclpy
from flask import Flask, jsonify, request

from .pathfinder_node import PathfinderNode

app = Flask(__name__)
pathfinder_node: PathfinderNode | None = None


@app.route("/", methods=["GET"])
def ping():
    return jsonify({"ping": "pong"})


@app.route("/go_forward", methods=["GET"])
def go_forward():
    pathfinder_node.update_params(speed=pathfinder_node.acceleration)
    return jsonify({"success": 200})


@app.route("/go_backwards", methods=["GET"])
def go_backwards():
    pathfinder_node.update_params(speed=-1 * pathfinder_node.acceleration)
    return jsonify({"success": 200})


@app.route("/turn_left", methods=["GET"])
def turn_left():
    pathfinder_node.update_params(steering=pathfinder_node.steering_accel)
    return jsonify({"success": 200})


@app.route("/turn_right", methods=["GET"])
def turn_right():
    pathfinder_node.update_params(steering=-1 * pathfinder_node.steering_accel)
    return jsonify({"success": 200})


@app.route("/run_test", methods=["POST"])
def run_test():
    data = request.get_json()
    testIndex, steering, speed = data["test"], data["steering"], data["speed"]
    if testIndex not in {1, 2, 3}:
        return jsonify({"error": "Index should be 1, 2, or 3"}, 400)

    if testIndex == 1:
        # Drive forwards for 5 seconds
        pathfinder_node.set_params(
            speed=50 * speed, steering=0.0 * steering
        )  # Go at speed=5
        time.sleep(5)
        pathfinder_node.set_params(speed=0.0 * speed)
    elif testIndex == 2:
        # Turn in a circle ish
        pathfinder_node.set_params(
            speed=50 * speed, steering=5 * steering
        )  # Go at speed=5
        time.sleep(20)
        pathfinder_node.set_params(speed=0.0 * speed, steering=0.0 * steering)
    else:
        # Figure-8 style motion
        pathfinder_node.set_params(
            speed=100.0 * speed, steering=0.0 * steering
        )  # Straight
        time.sleep(5)
        pathfinder_node.set_params(speed=50.0 * speed, steering=5.0  * steering)
        time.sleep(12)  # 3/4 turn
        pathfinder_node.set_params(
            speed=100.0 * speed, steering=0.0  * steering
        )  # Straight
        time.sleep(5)
        pathfinder_node.set_params(speed=50.0 * speed, steering=5.0  * steering)
        time.sleep(12)  # 3/4 turn
        pathfinder_node.set_params(
            speed=100.0 * speed, steering=0.0  * steering
        )  # Straight
        time.sleep(5)
        pathfinder_node.set_params(
            speed=0.0 * speed, steering=0.0  * steering
        )  # Straight
    return jsonify({"success": 200})


def start(node: PathfinderNode) -> None:
    rclpy.spin(node)


def main():
    global pathfinder_node

    rclpy.init()
    pathfinder_node = PathfinderNode()
    ros_thread = threading.Thread(target=start, args=(pathfinder_node,), daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=8001, debug=False)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
