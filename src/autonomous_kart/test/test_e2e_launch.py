"""
End-to-end tests against the full sim stack.

The stack is launched once per module (fixture `running_stack`) so every
test in this file amortises the ~10 s bringup cost:

  1. Synthesize a tiny mp4 if the real EVC footage isn't on disk.
  2. Spawn `ros2 launch autonomous_kart bringup_sim.launch.py`.
  3. Wait for localization (/odom) and for master_api on :8000 to
     answer `GET /`.
  4. Yield a context with: base URL, the subprocess, a rclpy listener
     already subscribed to /odom, /cmd_vel, /cmd_turn, /manual_commands.
  5. SIGINT the subprocess in teardown.

Tests cover both the HTTP API surface of `master_api` and the
end-to-end autonomous driving loop. Everything in one stack lifecycle.
"""
import json
import os
import shutil
import signal
import subprocess
import time
import urllib.error
import urllib.request

import pytest

rclpy_mod = pytest.importorskip("rclpy")

SIM_VIDEO_PATH = "/ws/data/EVC_test_footage/video.mp4"
BASE_URL = "http://127.0.0.1:8000"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _ensure_sim_video():
    if os.path.exists(SIM_VIDEO_PATH):
        return
    try:
        import cv2
        import numpy as np
    except ImportError:
        pytest.skip("opencv/numpy not available to synthesize test video")

    os.makedirs(os.path.dirname(SIM_VIDEO_PATH), exist_ok=True)
    w, h = 320, 180
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(SIM_VIDEO_PATH, fourcc, 30.0, (w, h))
    if not writer.isOpened():
        pytest.skip("cv2.VideoWriter could not open mp4 encoder")
    for i in range(60):
        frame = np.zeros((h, w, 3), dtype="uint8")
        frame[:, : w // 2] = (40, 40, 40)
        frame[:, w // 2 :] = (200, 200, 200)
        frame[h // 2 - 3 : h // 2 + 3, :] = (i * 4, 128, 255 - i * 4)
        writer.write(frame)
    writer.release()


def _ros_available():
    if shutil.which("ros2") is None:
        return False
    try:
        out = subprocess.check_output(
            ["ros2", "pkg", "prefix", "autonomous_kart"],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5,
        )
    except Exception:
        return False
    return bool(out.strip())


def _http_get(path, timeout=3.0):
    with urllib.request.urlopen(BASE_URL + path, timeout=timeout) as r:
        return r.status, json.loads(r.read().decode() or "null")


def _http_post(path, payload, timeout=3.0):
    body = json.dumps(payload).encode() if payload is not None else b""
    req = urllib.request.Request(
        BASE_URL + path,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode() or "null")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode() or "null")
        except Exception:
            return e.code, None


def _http_post_raw(path, raw_body, timeout=3.0):
    req = urllib.request.Request(
        BASE_URL + path,
        data=raw_body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()


# ---------------------------------------------------------------------------
# Module-scoped stack fixture
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def running_stack():
    if not _ros_available():
        pytest.skip("autonomous_kart not installed in this environment")

    _ensure_sim_video()

    env = os.environ.copy()
    env.setdefault("ROS_DOMAIN_ID", "77")

    proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "autonomous_kart",
            "bringup_sim.launch.py",
            "simulation_mode:=true",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )

    import rclpy
    import rclpy.executors  # noqa: F401

    rclpy.init(args=["--ros-args"])
    listener = None
    cmd_listener = None
    odom_listener = None
    try:
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32, Float32MultiArray

        # Split listeners: high-rate /odom gets its own node + executor so
        # it can't starve the low-rate manual_commands / cmd_vel / cmd_turn
        # callbacks. Also use depth=1 for odom — we only need "did it arrive",
        # not a backlog.
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        odom_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        odom_listener = rclpy.create_node("e2e_odom_listener")
        cmd_listener = rclpy.create_node("e2e_cmd_listener")
        listener = rclpy.create_node("e2e_api_listener")

        counters = {"odom": 0, "cmd_vel": 0, "cmd_turn": 0}
        manual_cmds = []

        odom_listener.create_subscription(
            Odometry, "odom",
            lambda m: counters.__setitem__("odom", counters["odom"] + 1),
            odom_qos,
        )
        cmd_listener.create_subscription(
            Float32, "cmd_vel",
            lambda m: counters.__setitem__("cmd_vel", counters["cmd_vel"] + 1), 10,
        )
        cmd_listener.create_subscription(
            Float32, "cmd_turn",
            lambda m: counters.__setitem__("cmd_turn", counters["cmd_turn"] + 1), 10,
        )
        listener.create_subscription(
            Float32MultiArray, "manual_commands",
            lambda m: manual_cmds.append(list(m.data)), 10,
        )

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(listener)
        executor.add_node(cmd_listener)
        executor.add_node(odom_listener)

        # Phase 1: wait for localization.
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline and counters["odom"] == 0:
            if proc.poll() is not None:
                out = proc.stdout.read() if proc.stdout else ""
                pytest.fail(
                    f"bringup_sim exited early (rc={proc.returncode})\n--- output ---\n{out}"
                )
            executor.spin_once(timeout_sec=0.1)
        if counters["odom"] == 0:
            pytest.fail("localization never published /odom within 25s")

        # Phase 2: wait for master_api HTTP to respond.
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                status, body = _http_get("/", timeout=1.0)
                if status == 200 and body == {"ping": "pong"}:
                    break
            except Exception:
                time.sleep(0.25)
        else:
            pytest.fail("master_api never responded on :8000")

        def pump(predicate, timeout=3.0):
            # Drain every ready callback on every node per iteration, not
            # just one. spin_once picks a single ready callback; under load
            # that lets a 60 Hz publisher starve a one-shot message.
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                for _ in range(20):  # drain burst
                    before = (
                        counters["odom"],
                        counters["cmd_vel"],
                        counters["cmd_turn"],
                        len(manual_cmds),
                    )
                    executor.spin_once(timeout_sec=0.0)
                    after = (
                        counters["odom"],
                        counters["cmd_vel"],
                        counters["cmd_turn"],
                        len(manual_cmds),
                    )
                    if before == after:
                        break  # nothing more ready, stop draining
                if predicate():
                    return True
                executor.spin_once(timeout_sec=0.02)  # wait for new traffic
            return False

        yield {
            "proc": proc,
            "rclpy": rclpy,
            "listener": listener,
            "counters": counters,
            "manual_cmds": manual_cmds,
            "pump": pump,
        }
    finally:
        try:
            for n in (listener, cmd_listener, odom_listener):
                if n is None:
                    continue
                try:
                    n.destroy_node()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=5)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# API tests — ordered so state transitions compound naturally.
# ---------------------------------------------------------------------------


def test_ping(running_stack):
    status, body = _http_get("/")
    assert status == 200
    assert body == {"ping": "pong"}


def test_initial_state_is_idle(running_stack):
    status, body = _http_get("/get_state")
    assert status == 200
    assert body["state"] == "IDLE"


def test_racing_line_endpoint_returns_points(running_stack):
    status, body = _http_get("/racing_line")
    assert status == 200
    assert "points" in body
    assert len(body["points"]) > 0
    assert len(body["points"][0]) == 2  # [x, y]


def test_get_logs_returns_list(running_stack):
    status, body = _http_get("/get_logs")
    assert status == 200
    assert isinstance(body, list)


def test_odom_endpoint_returns_snapshot(running_stack):
    # odom callback only fires after the topic publishes, so give the
    # master node a tick to ingest /odom.
    running_stack["pump"](lambda: running_stack["counters"]["odom"] > 0, timeout=2.0)
    status, body = _http_get("/odom")
    assert status == 200
    for key in ("x", "y", "yaw", "speed", "motor", "steer", "state"):
        assert key in body, f"missing {key} in /odom response"


def test_set_state_rejects_garbage(running_stack):
    status, body = _http_post("/set_state", {"state": "GARBAGE"})
    # master_api returns 200 with an error field for bad state values.
    assert status == 200
    assert "error" in body
    # State must be unchanged.
    _, body = _http_get("/get_state")
    assert body["state"] == "IDLE"


def test_set_state_missing_field(running_stack):
    status, body = _http_post("/set_state", {"not_state": "IDLE"})
    assert "error" in body
    _, body = _http_get("/get_state")
    assert body["state"] == "IDLE"


def test_set_state_invalid_json_body(running_stack):
    # Flask rejects malformed JSON with its built-in HTML 400 response
    # before the view ever runs, so we only assert the status code.
    status, _ = _http_post_raw("/set_state", b"this is not json")
    assert status == 400


def test_set_state_to_manual(running_stack):
    status, body = _http_post("/set_state", {"state": "MANUAL"})
    assert status == 200
    assert body.get("success") == "ok"
    _, body = _http_get("/get_state")
    assert body["state"] == "MANUAL"


def test_manual_control_publishes_on_topic(running_stack):
    before = len(running_stack["manual_cmds"])
    status, body = _http_post(
        "/manual_control", {"speed": 1.5, "steering": -2.5}
    )
    assert status == 200
    assert body.get("success") == "ok"
    assert running_stack["pump"](
        lambda: len(running_stack["manual_cmds"]) > before, timeout=2.0
    )
    assert running_stack["manual_cmds"][-1] == [1.5, -2.5]


def test_manual_control_rejects_missing_field(running_stack):
    status, body = _http_post("/manual_control", {"speed": 1.0})
    assert status == 400
    assert "error" in body


def test_manual_control_rejects_non_dict(running_stack):
    status, raw = _http_post_raw("/manual_control", b"[1, 2, 3]")
    assert status == 400
    assert "error" in raw


def test_set_state_to_autonomous_and_driving_loop(running_stack):
    # Snapshot counters before the switch so we can assert delta, not
    # anything an earlier test may have produced.
    counters = running_stack["counters"]
    vel0, turn0 = counters["cmd_vel"], counters["cmd_turn"]

    status, body = _http_post("/set_state", {"state": "AUTONOMOUS"})
    assert status == 200
    assert body.get("success") == "ok"

    ok = running_stack["pump"](
        lambda: counters["cmd_vel"] > vel0 and counters["cmd_turn"] > turn0,
        timeout=25.0,
    )
    if not ok:
        proc = running_stack["proc"]
        if proc.poll() is not None:
            out = proc.stdout.read() if proc.stdout else ""
            pytest.fail(f"launch died, rc={proc.returncode}\n{out}")
        pytest.fail("pathfinder never produced commands after state=AUTONOMOUS")

    # Process must still be alive at the end of the whole API suite.
    assert running_stack["proc"].poll() is None
    # /odom endpoint should now show AUTONOMOUS.
    _, body = _http_get("/odom")
    assert body["state"] == "AUTONOMOUS"
