import time
import traceback

import rclpy
import math

from nav_msgs.msg import Odometry
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

        self.imu_publisher = self.create_publisher(Odometry, "gps", 5)

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
        self.write_byte_data(self.i2c_address, 0x6B, 0x00)  # wake

        self.timer = self.create_timer(1.0 / self.imu_frequency, self.publish_imu)

    def publish_imu(self):
        pass

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
        node.running = False
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
