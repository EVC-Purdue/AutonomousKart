import time
import traceback

import rclpy
import math

from sensor_msgs.msg import Imu
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from smbus2 import SMBus

class ImuNode(Node):
    def __init__(self):
        super().__init__(
            "imu_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.sim_mode = self.get_parameter("simulation_mode").value

        self.imu_publisher = self.create_publisher(Imu, "imu", 10)

        self.last_callback_time = time.time()

        self.logger = self.get_logger()

        self.bus_num = int(self.get_parameter("i2c_bus").value)
        self.i2c_address = int(self.get_parameter("i2c_address").value)
        self.imu_frequency = float(self.get_parameter("imu_frequency").value)
        self.default_g = float(self.get_parameter("default_g").value)
        self.accel_var = float(self.get_parameter("accel_variance").value)
        self.gyro_var = float(self.get_parameter("gyro_variance").value)
        self.calib_samples = int(self.get_parameter("calibration_samples").value)
        self.accel_per_g = int(self.get_parameter("accel_per_g").value)
        self.gyro_lsb_per_dps = int(self.get_parameter("gyro_lsb_per_dps").value)

        self.gyro_bias = [0.0, 0.0, 0.0]
        self._calib_sum = [0.0, 0.0, 0.0]
        self._calib_count = 0
        self._calibrated = False

        self.logger.info(
            f"IMU Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

        if not self.imu_frequency or self.imu_frequency <= 0:
            self.imu_frequency = 100
            self.logger.warning(f"No IMU frequency known, setting to default={self.imu_frequency}")

        if self.sim_mode:
            # Shouldn't run
            pass

        self.bus = SMBus(self.bus_num)
        self.bus.write_byte_data(self.i2c_address, 0x6B, 0x00)  # wake

        self.timer = self.create_timer(1.0 / self.imu_frequency, self.publish_imu)

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

    def calibrate(self, gyro):
        """Average the first calib_samples gyro readings as bias. Kart
        must be still during this. Returns True when done."""
        for i in range(3):
            self._calib_sum[i] += gyro[i]
        self._calib_count += 1

        if self._calib_count >= self.calib_samples:
            self.gyro_bias = [s / self._calib_count for s in self._calib_sum]
            self._calibrated = True
            self.logger.info(f"Gyro bias: {self.gyro_bias}")
        return self._calibrated

    def publish_imu(self):
        try:
            accel, gyro = self.read_imu()
        except OSError as e:
            self.logger.warning(f"IMU read failed: {e}")
            return

        if not self._calibrated:
            self.calibrate(gyro)
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # No orientation knowledge
        msg.orientation_covariance[0] = 1e6

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