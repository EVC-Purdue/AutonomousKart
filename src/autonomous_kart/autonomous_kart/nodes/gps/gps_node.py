import time
import traceback
import serial
import rclpy
import math

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class GpsNode(Node):
    def __init__(self):
        super().__init__(
            "gps_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.sim_mode = self.get_parameter("simulation_mode").value

        self.gps_publisher = self.create_publisher(Odometry, "gps", 5)

        self.last_callback_time = time.time()

        self.logger = self.get_logger()

        self.gps_frequency = self.get_parameter("gps_frequency").value
        
        self.gps_data_pos: list[float] = [0.0, 0.0, 0.0]
        self.gps_data_quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Left as identity
        self.gps_data_cov: list[float] = [0.0] * 36
        self.gps_data_cov[21] = -1.0
        self.gps_data_cov[28] = -1.0
        self.gps_data_cov[35] = -1.0
        
        self.origin_lat = None
        self.origin_lon = None

        self.logger.info(
            f"Gps Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

        if not self.gps_frequency or self.gps_frequency <= 0:
            self.logger.warning("No GPS frequency known, setting to default")
            self.gps_frequency = 10

        self.logger.info(f"Gps Frequency: {self.gps_frequency}")

        # Note, need to add serial_device & baud_rate parameter
        self.device = self.get_parameter("serial_device").value or "/dev/ttyTHS1"
        self.baud = self.get_parameter("baud_rate").value or 9600

        self.ser = serial.Serial(self.device, baudrate=self.baud, timeout=0)
        self.buffer = ""

        self.timer = self.create_timer(1.0 / self.gps_frequency, self.publish_gps)

        
    def publish_gps(self):
        """
        Publishes the 2D coordinates and error
        """
        if not self.sim_mode:
            self.read_gps()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.gps_data_pos[0]
        msg.pose.pose.position.y = self.gps_data_pos[1]
        msg.pose.pose.position.z = self.gps_data_pos[2]

        msg.pose.pose.orientation = self.gps_data_quaternion

        msg.pose.covariance = self.gps_data_cov

        self.gps_publisher.publish(msg)

    def read_gps(self):
        if self.ser is None or not self.ser.is_open:
            return

        # read right now
        chunk = self.ser.read(self.ser.in_waiting or 1)
        if not chunk:
            return

        self.buffer += chunk.decode("ascii", errors="ignore")

        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            self.parse(line.strip())

    def parse(self, msg: str):
        if not msg.startswith("$"):
            return

        fields = msg.split(",")
        gng_type = fields[0][1:]

        if gng_type[:3] != "GNG":
            return

        match gng_type[3:]:
            case "GGA":
                self.handle_gga(fields)
            case "GSA":
                self.handle_gsa(fields)
            case "RMC":
                self.handle_rmc(fields)


    def handle_gga(self, fields):
        lat = fields[2]
        lat_direction = fields[3]

        lon = fields[4]
        lon_direction = fields[5]

        if not lat or not lon or not lat_direction or not lon_direction:
            return

        num_satellites = fields[7]
        hdop = fields[8]
        
        self.gps_to_coords(lat, lat_direction, lon, lon_direction)
        

    def handle_gsa(self, fields):
        if not fields[16] or not fields[17]:
            return
        
        hdop = float(fields[16]) ** 2
        vdop = float(fields[17]) ** 2

        self.gps_data_cov[0] = hdop
        self.gps_data_cov[7] = hdop
        self.gps_data_cov[14] = vdop
        self.gps_data_cov[21] = -1.0
        self.gps_data_cov[28] = -1.0
        self.gps_data_cov[35] = -1.0


    def handle_rmc(self, fields):
        lat = fields[3]
        lat_direction = fields[4]

        lon = fields[5]
        lon_direction = fields[6]

        if not lat or not lon or not lat_direction or not lon_direction:
            return

        self.gps_to_coords(lat, lat_direction, lon, lon_direction)


    def gps_to_coords(self, lat, lat_direction, lon, lon_direction):
        lon = self.nmea_to_decimal(lon, lon_direction)
        lat = self.nmea_to_decimal(lat, lat_direction)

        if not self.origin_lon:
            self.origin_lon = lon
            
        if not self.origin_lat:
            self.origin_lat = lat
           
        #An approx from earth global circ. to x, y
        R = 6_371_000
        self.gps_data_pos[0] = R * math.radians(lon - self.origin_lon) * math.cos(math.radians(self.origin_lat))
        self.gps_data_pos[1] = R * math.radians(lat - self.origin_lat)
    

    def nmea_to_decimal(self, value: str, direction: str) -> float:
        d = float(value)

        # nmea format is in ddmm.mmm or dddmm.mmm 
        # where d is degrees and m is decimal minutes

        degrees = int(d / 100)
        minutes = d - degrees * 100

        decimal = degrees + (minutes / 60.0)

        if direction in ('S', 'W'):
            decimal *= -1
        
        return decimal


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
