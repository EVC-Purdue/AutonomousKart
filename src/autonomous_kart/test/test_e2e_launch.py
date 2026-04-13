"""
End-to-end smoke test: bring up the full sim stack with `ros2 launch`,
wait for the driving loop to produce commands, assert nothing crashes.

Requires the installed package, so run this under `colcon test` or after
`colcon build` + sourcing install/setup.bash.

The camera sim reads an mp4 from /ws/data/EVC_test_footage/video.mp4.
If that file isn't present (fresh CI checkout), we synthesize a tiny
one so we don't have to ship a 75 MB asset.
"""
import os
import shutil
import signal
import subprocess
import time

import pytest

rclpy_mod = pytest.importorskip("rclpy")

SIM_VIDEO_PATH = "/ws/data/EVC_test_footage/video.mp4"


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


def test_full_stack_launches_and_drives(ros_ctx):
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

    try:
        with ros_ctx({}) as rclpy:
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Float32, String

            listener = rclpy.create_node("e2e_listener")
            got = {"odom": 0, "cmd_vel": 0, "cmd_turn": 0}
            listener.create_subscription(
                Odometry, "odom", lambda m: got.__setitem__("odom", got["odom"] + 1), 10
            )
            listener.create_subscription(
                Float32, "cmd_vel", lambda m: got.__setitem__("cmd_vel", got["cmd_vel"] + 1), 10
            )
            listener.create_subscription(
                Float32, "cmd_turn", lambda m: got.__setitem__("cmd_turn", got["cmd_turn"] + 1), 10
            )
            state_pub = listener.create_publisher(String, "system_state", 10)

            # Phase 1: wait for localization to come up.
            deadline = time.monotonic() + 25.0
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    out = proc.stdout.read() if proc.stdout else ""
                    pytest.fail(
                        f"bringup_sim exited early with code {proc.returncode}\n--- output ---\n{out}"
                    )
                rclpy.spin_once(listener, timeout_sec=0.1)
                if got["odom"] > 0:
                    break
            assert got["odom"] > 0, "localization never published /odom"

            # Phase 2: flip to AUTONOMOUS and wait for driving commands.
            for _ in range(10):
                state_pub.publish(String(data="AUTONOMOUS"))
                rclpy.spin_once(listener, timeout_sec=0.1)

            deadline = time.monotonic() + 25.0
            while time.monotonic() < deadline and (
                got["cmd_vel"] == 0 or got["cmd_turn"] == 0
            ):
                if proc.poll() is not None:
                    out = proc.stdout.read() if proc.stdout else ""
                    pytest.fail(
                        f"bringup_sim crashed mid-run, rc={proc.returncode}\n--- output ---\n{out}"
                    )
                rclpy.spin_once(listener, timeout_sec=0.1)

            assert got["cmd_vel"] > 0, "pathfinder never published cmd_vel"
            assert got["cmd_turn"] > 0, "pathfinder never published cmd_turn"
            assert proc.poll() is None, "launch process died before shutdown"
            listener.destroy_node()
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=5)
            except Exception:
                pass
