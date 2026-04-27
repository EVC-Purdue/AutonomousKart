import traceback
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mpu6050 import mpu6050
import numpy as np


class IMUNode(Node):
    def __init__(self):
        super().__init__(
            "imu_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.sim_mode = self.get_parameter("simulation_mode").value
        self.sim_mode = False

        self.imu_pub = self.create_publisher(Odometry, "imu_data", 5)

        self.poll_rate = self.get_parameter("poll_frequency").value
        self.timer = self.create_timer(1.0 / self.poll_rate, self.timer_callback)  # type: ignore

        self.imu_data_gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.imu_data_accel: tuple[float, float, float] = (0.0, 0.0, 0.0)

        self.get_logger().info(
            f"IMU Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

        self.sensor = None
        if not self.sim_mode:
            self.sensor = mpu6050(0x68)

        sample_time = self.get_parameter("gravity_comp_time").value
        self.calibrationSampleNum = int(sample_time * self.poll_rate) # type: ignore
        caliList = np.zeros((3, self.calibrationSampleNum)).tolist()
        self.needsCalibration: list[list[float]] | None = caliList
        self.gravity = (0.0, 0.0, 0.0)

    def timer_callback(self):
        if self.sim_mode:
            self.sim_poll()
        else:
            self.real_poll()

        imu_msg = Odometry()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link" # Unknown source of this field
        imu_msg.child_frame_id = "imu_link"

        imu_msg.twist.twist.angular.x = float(self.imu_data_gyro[0])
        imu_msg.twist.twist.angular.y = float(self.imu_data_gyro[1])
        imu_msg.twist.twist.angular.z = float(self.imu_data_gyro[2])

        imu_msg.twist.twist.linear.x = float(self.imu_data_accel[0])
        imu_msg.twist.twist.linear.y = float(self.imu_data_accel[1])
        imu_msg.twist.twist.linear.z = float(self.imu_data_accel[2])

        self.imu_pub.publish(imu_msg)
        # self.get_logger().info(f"Published IMU data: {imu_msg.data}")

    def real_poll(self):
        """Real mode - read IMU data"""

        if self.sensor is None:
            return

        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        if not self.needsCalibration:
            # Default codepath
            gyro_compensated = (
                gyro["x"] - self.gravity[0],
                gyro["y"] - self.gravity[1],
                gyro["z"] - self.gravity[2],
            )
            self.imu_data_gyro = gyro_compensated
            self.imu_data_accel = (accel["x"], accel["y"], accel["z"])  # type: ignore
            return

        # Calibration codepath
        self.calibrationSampleNum -= 1
        self.needsCalibration[0][self.calibrationSampleNum] = gyro["x"]
        self.needsCalibration[1][self.calibrationSampleNum] = gyro["y"]
        self.needsCalibration[2][self.calibrationSampleNum] = gyro["z"]
        if self.calibrationSampleNum == 0:
            x, y, z = self.needsCalibration
            self.gravity = (sum(x) / len(x), sum(y) / len(y), sum(z) / len(z))
            self.needsCalibration = None
            self.get_logger().info(
                f"IMU Calibration complete - Gravity: {self.gravity}"
            )

        # No acceleration compensation
        self.imu_data_gyro = (0.0, 0.0, 0.0)
        self.imu_data_accel = (accel["x"], accel["y"], accel["z"])  # type: ignore

    def sim_poll(self):
        self.imu_data_gyro = (0.0, 0.0, 0.0)
        self.imu_data_accel = (0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)

    node = IMUNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
