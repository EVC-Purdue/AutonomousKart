"""
Tests for GpsNode's publish gating.

Position and covariance are node state written by separate NMEA handlers, so a
receiver with no fix used to publish (0, 0) with a zero covariance that
localization_node's sigma floor then read as a 2 cm fix.
"""
import sys
import types

import pytest

rclpy_mod = pytest.importorskip("rclpy")

# pyserial is a Pi-only runtime dep. Stub it before importing the node so the
# module loads on dev/CI machines where it isn't installed.
if "serial" not in sys.modules:
    _stub = types.ModuleType("serial")

    class _StubSerial:
        def __init__(self, *_a, **_k): pass

    _stub.Serial = _StubSerial
    sys.modules["serial"] = _stub

from autonomous_kart.nodes.gps.gps_node import GpsNode  # noqa: E402


# fix quality 0, no lat/lon: what the receiver sends before it has a solution.
NO_FIX_GGA = "$GNGGA,182053.00,,,,,0,00,99.99,,M,,M,,*54"
RTK_GGA = (
    "$GNGGA,182053.00,4026.35000,N,08656.70000,W,4,12,0.70,187.5,M,-33.8,M,1.0,0000*4F"
)


def _params():
    return {
        "simulation_mode": True,
        "gps_frequency": 20,
        "lat0": 40.4380475,
        "lon0": -86.9442826,
        "serial_device": "/dev/null",
        "baud_rate": 115200,
        "status_frequency": 10,
        "serial_poll_hz": 200,
        "configure_receiver": False,
        "receiver_port": "COM3",
        "hdg_sigma_deg": 0.164,
        "hdg_accept_status": ["SOL_COMPUTED"],
        "vtg_speed_sigma_mps": 0.1,
        "vtg_min_speed_for_yaw": 0.5,
    }


@pytest.fixture
def gps_rig(ros_ctx, spin_helper):
    """Yield (node, received, pump); the node's own timers are cancelled."""
    from nav_msgs.msg import Odometry

    with ros_ctx(_params()) as rclpy:
        node = GpsNode()
        node.timer.cancel()
        node.status_timer.cancel()
        driver = rclpy.create_node("gps_driver")
        received = []
        driver.create_subscription(Odometry, "gps", lambda m: received.append(m), 10)
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery

            def pump(n=1):
                before = len(received)
                spin_helper(exe, lambda: len(received) >= before + n, timeout=0.5)

            yield node, received, pump
        finally:
            exe.remove_node(driver)
            driver.destroy_node()
            node.destroy_node()


def test_gps_withholds_message_when_receiver_has_no_fix(gps_rig):
    node, received, pump = gps_rig
    node.parse(NO_FIX_GGA)
    node.publish_gps()
    pump()
    assert received == [], "no-fix GGA must not produce a /gps message"


def test_gps_publishes_once_the_receiver_has_a_fix(gps_rig):
    node, received, pump = gps_rig
    node.parse(RTK_GGA)
    node.publish_gps()
    pump()

    assert len(received) == 1
    p = received[0].pose.pose.position
    assert abs(p.x) > 1.0 and abs(p.y) > 1.0, f"expected a real position, got {p}"
    assert received[0].pose.covariance[0] > 0.0
    assert received[0].pose.covariance[7] > 0.0


def test_gps_withholds_again_after_losing_the_fix(gps_rig):
    node, received, pump = gps_rig
    node.parse(RTK_GGA)
    node.publish_gps()
    pump()
    assert len(received) == 1

    node.parse(NO_FIX_GGA)
    node.publish_gps()
    pump()
    assert len(received) == 1, "a lost fix must not keep republishing the last position"
