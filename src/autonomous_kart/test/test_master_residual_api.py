"""Tests for the residual API plumbing in master_node + master_api."""
import pytest

rclpy = pytest.importorskip("rclpy")  # skip whole file on machines w/o rclpy
from std_msgs.msg import Float32MultiArray


def _status_msg_with_train_seq(seq, val_mae_s=0.10, val_mae_d=0.03):
    """Build an 88-float /mpc/status message with the train_seq slot set."""
    data = [0.0] * 88
    data[0] = 0.0    # mode
    data[1] = 1.0    # success
    data[67] = float(seq)   # train_seq
    data[70] = float(val_mae_s)
    data[71] = float(val_mae_d)
    data[72] = float(val_mae_s + 0.05)  # rls_val_mae_s
    data[73] = float(val_mae_d + 0.02)  # rls_val_mae_d
    data[74] = 1.0   # active_model = GBM
    m = Float32MultiArray()
    m.data = data
    return m


def test_residual_log_records_new_train_seq(ros_ctx):
    from autonomous_kart.nodes.master.master_node import MasterNode
    with ros_ctx():
        node = MasterNode()
        try:
            node._mpc_status_callback(_status_msg_with_train_seq(seq=1))
            node._mpc_status_callback(_status_msg_with_train_seq(seq=1))
            node._mpc_status_callback(_status_msg_with_train_seq(seq=2, val_mae_s=0.08))
            events = node.get_residual_log(limit=10)
            assert len(events) == 2
            assert events[0]["train_seq"] == 2     # newest first
            assert events[1]["train_seq"] == 1
        finally:
            node.destroy_node()


def test_residual_log_limit(ros_ctx):
    from autonomous_kart.nodes.master.master_node import MasterNode
    with ros_ctx():
        node = MasterNode()
        try:
            for s in range(5):
                node._mpc_status_callback(_status_msg_with_train_seq(seq=s + 1))
            events = node.get_residual_log(limit=2)
            assert len(events) == 2
            assert events[0]["train_seq"] == 5
            assert events[1]["train_seq"] == 4
        finally:
            node.destroy_node()


def test_residual_status_returns_latest_snapshot(ros_ctx):
    from autonomous_kart.nodes.master.master_node import MasterNode
    with ros_ctx():
        node = MasterNode()
        try:
            node._mpc_status_callback(_status_msg_with_train_seq(seq=42, val_mae_s=0.07))
            snap = node.get_residual_status()
            assert snap is not None
            assert snap["train_seq"] == 42
            assert abs(snap["last_val_mae_s"] - 0.07) < 1e-6
        finally:
            node.destroy_node()
