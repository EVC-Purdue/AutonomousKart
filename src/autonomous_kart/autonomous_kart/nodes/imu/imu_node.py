import json
import math
import os
import time
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float32, String

from smbus2 import SMBus


WAITING = "WAITING"
CALIBRATING = "CALIBRATING"
CALIBRATED = "CALIBRATED"


class ImuNode(Node):
    def __init__(self):
        super().__init__(
            "imu_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.sim_mode = self.get_parameter("simulation_mode").value
        self.logger = self.get_logger()

        self.bus_num = int(self.get_parameter("i2c_bus").value)
        self.i2c_address = int(self.get_parameter("i2c_address").value)
        self.imu_frequency = float(self.get_parameter("imu_frequency").value)
        self.default_g = float(self.get_parameter("default_g").value)
        self.accel_var = float(self.get_parameter("accel_variance").value)
        self.gyro_var = float(self.get_parameter("gyro_variance").value)
        self.calib_samples = int(self.get_parameter("calibration_samples").value)
        self.accel_per_g = float(self.get_parameter("accel_per_g").value)
        self.gyro_lsb_per_dps = float(self.get_parameter("gyro_lsb_per_dps").value)

        self.gyro_motion_thresh = float(self.get_parameter("gyro_motion_thresh").value)
        self.accel_motion_thresh = float(self.get_parameter("accel_motion_thresh").value)
        self.cmd_vel_zero_eps = float(self.get_parameter("cmd_vel_zero_eps").value)
        self.cmd_vel_max_age_s = float(self.get_parameter("cmd_vel_max_age_s").value)
        self.cache_path = str(self.get_parameter("calibration_cache_path").value)
        self.status_frequency = float(self.get_parameter("status_frequency").value)

        if not self.imu_frequency or self.imu_frequency <= 0:
            self.imu_frequency = 100.0
            self.logger.warning(f"No IMU frequency known, defaulting to {self.imu_frequency}")

        self.gyro_bias = [0.0, 0.0, 0.0]
        self._calib_sum = [0.0, 0.0, 0.0]
        self._calib_count = 0
        self.state = WAITING
        self.last_error = ""

        # cmd_vel tracking; None = never received one
        self._last_cmd_vel = None
        self._last_cmd_vel_t = 0.0

        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        self.status_publisher = self.create_publisher(String, "imu/calibration_status", 1)

        self.create_subscription(Float32, "cmd_vel", self._cmd_vel_callback, 5)
        self.create_subscription(Empty, "imu/calibrate", self._calibrate_trigger, 1)

        self.logger.info(f"IMU Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}")

        if self.sim_mode:
            self.logger.error("IMU should not run on SIM")
            return

        self.bus = SMBus(self.bus_num)
        self.bus.write_byte_data(self.i2c_address, 0x6B, 0x00)  # wake

        self._try_load_cache()

        self.create_timer(1.0 / self.imu_frequency, self.publish_imu)
        self.create_timer(1.0 / self.status_frequency, self._publish_status)
        self._publish_status()

    def read_imu(self):
        """Burst-read 14 bytes: accel(6) + temp(2) + gyro(6). Returns accel in m/s^2 and gyro in rad/s."""
        raw = self.bus.read_i2c_block_data(self.i2c_address, 0x3B, 14)

        def s16(hi, lo):
            """ Convert hi, low into one 2's complement int16 """
            v = (hi << 8) | lo
            return v - 65536 if v >= 32768 else v

        accel = tuple(
            s16(raw[i], raw[i + 1]) / self.accel_per_g * self.default_g
            for i in (0, 2, 4)
        )
        deg2rad = math.pi / 180.0

        gyro = tuple(
            s16(raw[i], raw[i + 1]) / self.gyro_lsb_per_dps * deg2rad
            for i in (8, 10, 12)
        )
        return accel, gyro

    def _cmd_vel_callback(self, msg: Float32):
        self._last_cmd_vel = float(msg.data)
        self._last_cmd_vel_t = time.time()

    def _calibrate_trigger(self, _msg: Empty):
        self.logger.info("Calibration trigger received - resetting calibration state")
        self._reset_calibration("triggered")

    def _reset_calibration(self, reason: str):
        self.state = WAITING
        self._calib_sum = [0.0, 0.0, 0.0]
        self._calib_count = 0
        self.last_error = reason
        self._publish_status()

    def _cmd_vel_is_zero(self) -> bool:
        # First-boot if we never recieved cmd_vel, assume idle.
        if self._last_cmd_vel is None:
            return True
        if (time.time() - self._last_cmd_vel_t) > self.cmd_vel_max_age_s:
            self.last_error = "cmd_vel stale"
            return False
        if abs(self._last_cmd_vel) > self.cmd_vel_zero_eps:
            self.last_error = f"cmd_vel non-zero ({self._last_cmd_vel:.3f})"
            return False
        return True

    def _sample_indicates_motion(self, accel, gyro):
        for axis, val in zip(("x", "y", "z"), gyro):
            if abs(val) > self.gyro_motion_thresh:
                return f"gyro {axis}={val:.3f} rad/s exceeds {self.gyro_motion_thresh}"
        accel_mag = math.sqrt(sum(a * a for a in accel))
        if abs(accel_mag - abs(self.default_g)) > self.accel_motion_thresh:
            return f"|accel|={accel_mag:.3f} m/s^2 deviates from g by > {self.accel_motion_thresh}"
        return None

    def _accumulate(self, gyro):
        for i in range(3):
            self._calib_sum[i] += gyro[i]
        self._calib_count += 1

    def _finish_calibration(self):
        self.gyro_bias = [s / self._calib_count for s in self._calib_sum]
        self.state = CALIBRATED
        self.last_error = ""
        self.logger.info(
            f"Calibration complete after {self._calib_count} samples. gyro_bias={self.gyro_bias}"
        )
        self._save_cache()
        self._publish_status()

    def _try_load_cache(self):
        if not os.path.exists(self.cache_path):
            self.logger.info(f"No calibration cache at {self.cache_path}; will calibrate fresh")
            return
        try:
            with open(self.cache_path, "r") as f:
                data = json.load(f)
            bias = data["gyro_bias"]
            if not (isinstance(bias, list) and len(bias) == 3):
                raise ValueError("gyro_bias must be a length-3 list")
            self.gyro_bias = [float(b) for b in bias]
            self.state = CALIBRATED
            self.logger.info(
                f"Loaded calibration from {self.cache_path}: gyro_bias={self.gyro_bias}"
            )
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as e:
            self.logger.warning(
                f"Cache file at {self.cache_path} unreadable ({e}); will calibrate fresh"
            )

    def _save_cache(self):
        payload = {
            "gyro_bias": self.gyro_bias,
            "samples": self._calib_count,
            "timestamp": time.time(),
        }
        tmp = self.cache_path + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(payload, f)
            os.replace(tmp, self.cache_path)
            self.logger.debug(f"Saved calibration to {self.cache_path}")
        except OSError as e:
            self.logger.warning(f"Failed to save calibration cache: {e}")

    def publish_imu(self):
        try:
            accel, gyro = self.read_imu()
        except OSError as e:
            self.logger.warning(f"IMU read failed: {e}")
            return

        if self.state == CALIBRATED:
            self._publish_imu_msg(accel, gyro)
            return

        if self.state == WAITING:
            if not self._cmd_vel_is_zero():
                return
            self.state = CALIBRATING
            self.logger.info("Starting calibration")

        # CALIBRATING
        motion_reason = self._sample_indicates_motion(accel, gyro)
        if motion_reason is not None:
            self.logger.warning(f"Calibration aborted: {motion_reason}")
            self._reset_calibration(motion_reason)
            return

        self._accumulate(gyro)
        if self._calib_count >= self.calib_samples:
            self._finish_calibration()

    def _publish_imu_msg(self, accel, gyro):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.orientation_covariance[0] = 1e6  # no orientation knowledge

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.linear_acceleration_covariance[0] = self.accel_var
        msg.linear_acceleration_covariance[4] = self.accel_var
        msg.linear_acceleration_covariance[8] = self.accel_var

        msg.angular_velocity.x = gyro[0] - self.gyro_bias[0]
        msg.angular_velocity.y = gyro[1] - self.gyro_bias[1]
        msg.angular_velocity.z = gyro[2] - self.gyro_bias[2]
        msg.angular_velocity_covariance[0] = self.gyro_var
        msg.angular_velocity_covariance[4] = self.gyro_var
        msg.angular_velocity_covariance[8] = self.gyro_var

        self.imu_publisher.publish(msg)

    def _publish_status(self):
        payload = {
            "state": self.state,
            "samples": self._calib_count,
            "target_samples": self.calib_samples,
            "gyro_bias": self.gyro_bias,
            "last_error": self.last_error,
            "cache_path": self.cache_path,
        }
        self.status_publisher.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)

    node = ImuNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
