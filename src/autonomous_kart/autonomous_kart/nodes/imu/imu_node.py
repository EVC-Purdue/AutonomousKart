import traceback
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
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

        self.imu_pub = self.create_publisher(Float32MultiArray, "imu_data", 5)

        self.imu_layout = [
            MultiArrayDimension(label="rows", size=2, stride=6),
            MultiArrayDimension(label="cols", size=3, stride=3),
        ]

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

    def timer_callback(self):
        if self.sim_mode:
            self.sim_poll()
        else:
            self.real_poll()

        # Format data into a 2x3 array
        imu_msg = Float32MultiArray()
        imu_msg.layout.dim = self.imu_layout
        imu_msg.layout.data_offset = 0
        imu_msg.data = [
            self.imu_data_gyro[0],
            self.imu_data_gyro[1],
            self.imu_data_gyro[2],
            ######################
            self.imu_data_accel[0],
            self.imu_data_accel[1],
            self.imu_data_accel[2],
        ]

        # Output as a 2x3 float array:
        # [[accel_x, accel_y, accel_z], [gyro_x, gyro_y, gyro_z]]
        #  [forward,  left,     up   ], [ roll ,  pitch,  yaw  ]
        self.imu_pub.publish(imu_msg)
        # self.get_logger().info(f"Published IMU data: {imu_msg.data}")

    def real_poll(self):
        """Real mode - read IMU data"""

        if self.sensor is None:
            return

        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        self.imu_data_gyro = (gyro["x"], gyro["y"], gyro["z"])
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
