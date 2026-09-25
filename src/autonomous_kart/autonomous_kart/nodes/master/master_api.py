import threading
import os, csv, math, re
import logging

import rclpy
from rclpy.executors import ExternalShutdownException
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS

from autonomous_kart import paths
from .master_node import MasterNode, STATES

logging.getLogger("werkzeug").setLevel(logging.ERROR)

app = Flask(__name__)
CORS(app)
master_node: MasterNode | None = None

@app.route("/", methods=["GET"])
def ping():
    return jsonify({"ping": "pong"})

_STATIC_LINE_CACHE = {}

def _load_static_line(path):
    """
    Load racing line once per path
    returns: list of full waypoint dicts
    """
    cached = _STATIC_LINE_CACHE.get(path)
    if cached is not None:
        return cached
    if not os.path.exists(path):
        return []
    rows = []
    with open(path, "r") as f:
        for row in f:
            parts = row.strip().split(",")
            if len(parts) >= 6:
                try:
                    rows.append({
                        "s": float(parts[0]),
                        "x": float(parts[1]),
                        "y": float(parts[2]),
                        "psi": float(parts[3]),
                        "kappa": float(parts[4]),
                        "vx": float(parts[5]),
                    })
                except ValueError:
                    continue
    _STATIC_LINE_CACHE[path] = rows
    return rows

_TRACK_EDGE_CACHE = {}
_PLACEMARK_RE = re.compile(
    r"<Placemark[^>]*>.*?<name>(.*?)</name>.*?<coordinates>(.*?)</coordinates>",
    re.S,
)
_R_EARTH = 6_371_000  # gps_node.gps_to_coords

def _load_track_edges(path, lat0, lon0):
    """
    Load the surveyed asphalt boundaries once, projected into the /odom frame
    about (lat0, lon0) the same way gps_node projects a fix.
    returns: {"inner": [[x, y], ...], "outer": [[x, y], ...]}
    """
    cached = _TRACK_EDGE_CACHE.get(path)
    if cached is not None:
        return cached
    edges = {"inner": [], "outer": []}
    if not os.path.exists(path):
        return edges
    cos_lat0 = math.cos(math.radians(lat0))
    for name, coords in _PLACEMARK_RE.findall(open(path, "r").read()):
        key = name.strip().lower()
        side = "inner" if key.startswith("inside") else (
            "outer" if key.startswith("outside") else None)
        if side is None:
            continue
        for tok in coords.split():
            parts = tok.split(",")
            if len(parts) < 2:
                continue
            try:
                lon, lat = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            edges[side].append([
                _R_EARTH * math.radians(lon - lon0) * cos_lat0,
                _R_EARTH * math.radians(lat - lat0),
            ])
    _TRACK_EDGE_CACHE[path] = edges
    return edges

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


@app.route("/e_comms", methods=["GET"])
def e_comms():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_e_comms())


@app.route("/mpc_status", methods=["GET"])
def mpc_status():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_mpc_status())


@app.route("/rl_residual_status", methods=["GET"])
def rl_residual_status():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_rl_residual_status())


@app.route("/mpc/residual_mode", methods=["POST"])
def mpc_residual_mode():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    data = request.get_json(silent=True) or {}
    ok, reason = master_node.set_residual_mode(data.get("mode", ""))
    if not ok:
        return jsonify({"error": reason}), 400
    return jsonify({"success": "ok", "mode": reason})


@app.route("/mpc/actuator_gain", methods=["GET"])
def mpc_actuator_gain_get():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_actuator_gain())


@app.route("/mpc/actuator_gain", methods=["POST"])
def mpc_actuator_gain_set():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    data = request.get_json(silent=True) or {}
    ok, reason = master_node.set_actuator_gain(data.get("value"))
    if not ok:
        return jsonify({"error": reason}), 400
    return jsonify({"success": "ok", "value": reason})


@app.route("/pathfinder/planner", methods=["POST"])
def pathfinder_planner():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    data = request.get_json(silent=True) or {}
    ok, reason = master_node.set_planner(data.get("planner", ""))
    if not ok:
        return jsonify({"error": reason}), 400
    return jsonify({"success": "ok", "planner": reason})


@app.route("/pathfinder/line_path", methods=["POST"])
def pathfinder_line_path():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    data = request.get_json(silent=True) or {}
    ok, reason = master_node.set_line(data.get("path", ""))
    if not ok:
        return jsonify({"error": reason}), 400
    return jsonify({"success": "ok", "path": reason})


@app.route("/pathfinder/line_shapes", methods=["GET"])
def pathfinder_line_shapes():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_line_shapes())


@app.route("/pathfinder/line_speed", methods=["POST"])
def pathfinder_line_speed():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    data = request.get_json(silent=True) or {}
    ok, result = master_node.set_line_speed(data)
    if not ok:
        return jsonify({"error": result}), 400
    return jsonify({"success": "ok", "spec": result, "path": master_node.path})


@app.route("/gps", methods=["GET"])
def gps_status():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_gps_status())


@app.route("/imu/calibrate", methods=["POST"])
def imu_calibrate():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    master_node.trigger_imu_calibration()
    return jsonify({"success": "ok"})


@app.route("/imu/status", methods=["GET"])
def imu_status():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_imu_status())


@app.route("/imu", methods=["GET"])
def imu():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    return jsonify(master_node.get_imu())


@app.route("/racing_line", methods=["GET"])
def racing_line():
    path = master_node.path
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
    return send_from_directory(paths.resolve("viz"), "viz.html")

@app.route("/map", methods=["GET"])
def map_endpoint():
    waypoints = _load_static_line(master_node.path)
    if not waypoints:
        return jsonify({"error": "racing line not found", "waypoints": []}), 404
    return jsonify({
        "path": master_node.path,
        "count": len(waypoints),
        "waypoints": waypoints,
    })


@app.route("/track_edges", methods=["GET"])
def track_edges_endpoint():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    edges = _load_track_edges(master_node.track_path, master_node.lat0,
                              master_node.lon0)
    if not edges["inner"] and not edges["outer"]:
        return jsonify({"error": "track not found", **edges}), 404
    return jsonify({"path": master_node.track_path, **edges})


@app.route("/lines", methods=["GET"])
def lines_endpoint():
    if not master_node:
        return jsonify({"error": "not initialized"}), 500
    static_xy = [[w["x"], w["y"]] for w in _load_static_line(master_node.path)]
    return jsonify({
        "static": static_xy,
        "dynamic": master_node.get_dynamic_line(),
    })


@app.route("/residual/status", methods=["GET"])
def residual_status():
    if not master_node:
        return jsonify({"error": "master node not initialized"}), 500
    snap = master_node.get_residual_status()
    if snap is None:
        return jsonify({"status": "no /mpc/status received yet"}), 200
    return jsonify(snap), 200


@app.route("/residual/log", methods=["GET"])
def residual_log():
    if not master_node:
        return jsonify({"error": "master node not initialized"}), 500
    try:
        limit = int(request.args.get("limit", 50))
    except ValueError:
        limit = 50
    return jsonify({"events": master_node.get_residual_log(limit=limit)}), 200


@app.route("/residual/log/stream", methods=["GET"])
def residual_log_stream():
    if not master_node:
        return ("master node not initialized", 500)
    from flask import Response
    import json as _json
    import time as _time

    def gen():
        last_seen = -1
        while True:
            events = master_node.get_residual_log(limit=200)
            # events are newest-first; emit oldest-of-the-new-ones first
            new = [e for e in reversed(events) if e["train_seq"] > last_seen]
            for e in new:
                yield f"event: train\ndata: {_json.dumps(e)}\n\n"
                last_seen = e["train_seq"]
            _time.sleep(1.0)

    return Response(gen(), mimetype="text/event-stream")


@app.route("/residual/revert", methods=["POST"])
def residual_revert():
    if not master_node:
        return jsonify({"error": "master node not initialized"}), 500
    master_node.trigger_residual_revert()
    return ("", 202)


def start(node: MasterNode) -> None:
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Take the whole process down so Flask doesn't keep serving stale ROS state.
        os._exit(0)


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
