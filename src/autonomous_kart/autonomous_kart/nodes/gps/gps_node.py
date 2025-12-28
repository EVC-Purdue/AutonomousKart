import time
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Float32MultiArray


class GpsNode(Node):
    def __init__(self):
        super().__init__(
            "gps_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.create_publisher(Float32MultiArray, 'gps', 5)

        self.last_callback_time = time.time()
        self.logger = self.get_logger()

        self.gps_frequency = self.get_parameter("gps_frequency").value
        self.sim_mode = self.get_parameter("simulation_mode").value

        self.logger.info(f"Gps Frequency: {self.gps_frequency}")
        self.timer = self.create_timer(1.0 / self.gps_frequency, self.publish_gps)

        self.logger.info(
            f"Gps Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

    def get_gps_data(self):
        """ Gets GPS data with RTK on kart """
        # TODO: Implement functionality
        return 0, 0, 0, 0

    def publish_gps(self):
        """
        Publishes the 2D coordinates and error
        """
        if not self.sim_mode:
            long, lat, long_err, lat_err = self.get_gps_data()
            self.gps_publisher.publish(Float32MultiArray(data=[long, lat, long_err, lat_err]))
        else:
            pass

def main(args=None):
    rclpy.init(args=args)

    node = GpsNode()
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
