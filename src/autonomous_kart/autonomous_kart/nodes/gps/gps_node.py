import json
import threading
import socket
import time
import traceback
import serial
import rclpy
import math

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


# NMEA GGA fix quality codes
FIX_LABELS = {
    0: "INVALID",
    1: "GPS",
    2: "DGPS",
    3: "PPS",
    4: "RTK_FIXED",
    5: "RTK_FLOAT",
    6: "ESTIMATED",
    7: "MANUAL",
    8: "SIMULATION",
    9: "WAAS",
}


class GpsNode(Node):
    def __init__(self):
        super().__init__(
            "gps_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.sim_mode = self.get_parameter("simulation_mode").value

        self.gps_publisher = self.create_publisher(Odometry, "gps", 5)
        self.gps_status_publisher = self.create_publisher(String, "gps/status", 1)

        # GPS status snapshot; populated from NMEA messages and the RTCM loop.
        self.fix_quality = 0
        self.num_satellites = 0
        self.hdop = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.altitude = 0.0
        self.sigma_e = 0.0
        self.sigma_n = 0.0
        self.sigma_u = 0.0
        self.rtcm_bytes_total = 0
        self.rtcm_last_t = 0.0  # 0 until first RTCM byte arrives

        self.last_callback_time = time.time()

        self.logger = self.get_logger()

        self.gps_frequency = self.get_parameter("gps_frequency").value
        
        self.gps_data_pos: list[float] = [0.0, 0.0, 0.0]
        self.gps_data_quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Left as identity
        self.gps_data_cov: list[float] = [0.0] * 36
        self.gps_data_cov[21] = 1e6  # roll
        self.gps_data_cov[28] = 1e6  # pitch
        self.gps_data_cov[35] = 1e6  # yaw

        self.origin_lat = float(self.get_parameter("lat0").value)
        self.origin_lon = float(self.get_parameter("lon0").value)
        self.use_gst = False

        self.logger.info(
            f"Gps Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

        if not self.gps_frequency or self.gps_frequency <= 0:
            self.logger.warning("No GPS frequency known, setting to default")
            self.gps_frequency = 10

        # Note, need to add serial_device & baud_rate parameter
        self.device = self.get_parameter("serial_device").value or "/dev/ttyTHS1"
        self.baud = self.get_parameter("baud_rate").value or 9600

        if not self.sim_mode:
            self.ser = serial.Serial(self.device, baudrate=self.baud, timeout=0)
            threading.Thread(target=self._rtcm_loop, daemon=True).start()

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
        self.logger.debug(f"GPS Pos: {msg.pose.pose.position}")
        self.logger.debug(f"GPS Cov: {msg.pose.covariance}")
        self.gps_publisher.publish(msg)
        self.publish_status()

    def publish_status(self):
        """Publish a JSON snapshot of fix quality / RTK / measurements."""
        now = time.time()
        rtcm_age = (now - self.rtcm_last_t) if self.rtcm_last_t > 0.0 else None
        payload = {
            "fix_quality": self.fix_quality,
            "fix_label": FIX_LABELS.get(self.fix_quality, f"UNKNOWN({self.fix_quality})"),
            "rtk": self.fix_quality in (4, 5),
            "rtk_fixed": self.fix_quality == 4,
            "num_satellites": self.num_satellites,
            "hdop": self.hdop,
            "lat": self.lat,
            "lon": self.lon,
            "altitude": self.altitude,
            "x": self.gps_data_pos[0],
            "y": self.gps_data_pos[1],
            "sigma_e": self.sigma_e,
            "sigma_n": self.sigma_n,
            "sigma_u": self.sigma_u,
            "use_gst": self.use_gst,
            "rtcm_bytes_total": self.rtcm_bytes_total,
            "rtcm_last_age_s": rtcm_age,
            "stamp_ns": self.get_clock().now().nanoseconds,
        }
        self.gps_status_publisher.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def read_gps(self):
        if self.ser is None or not self.ser.is_open:
            self.logger.info("Serial closed")
            return

        # read right now
        chunk = self.ser.read(self.ser.in_waiting or 1)
        if not chunk:
            self.logger.debug("No chunk")
            return

        self.buffer += chunk.decode("ascii", errors="ignore")

        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            try:
                self.parse(line.strip())
            except Exception as e:
                self.logger.error(f"parse crash on '{line}': {e}")

    def parse(self, msg: str):
        if not msg.startswith("$"):
            return
        if "*" in msg:
            msg = msg.rsplit("*", 1)[0]  # Remove checksum

        fields = msg.split(",")
        gng_type = fields[0][1:]

        if len(gng_type) < 5 or gng_type[:2] not in ("GN", "GP"):
            return

        match gng_type[2:]:
            case "GGA":
                self.handle_gga(fields)
            case "GSA":
                # self.handle_gsa(fields)
                pass
            case "GST":
                self.handle_gst(fields)
            case "RMC":
                self.handle_rmc(fields)

    def handle_gga(self, fields):
        if len(fields) < 10:
            return

        self.logger.debug(f"GGA entered, len={len(fields)}, fix={fields[6] if len(fields) > 6 else '?'}")
        lat = fields[2]
        lat_direction = fields[3]
        lon = fields[4]
        lon_direction = fields[5]
        fix_quality = int(fields[6]) if fields[6] else 0
        num_satellites = fields[7]
        hdop = float(fields[8]) if fields[8] else 0.0
        altitude = float(fields[9]) if fields[9] else 0.0

        self.fix_quality = fix_quality
        try:
            self.num_satellites = int(num_satellites) if num_satellites else 0
        except ValueError:
            self.num_satellites = 0
        self.hdop = hdop
        self.altitude = altitude

        if not lat or not lon or not lat_direction or not lon_direction:
            return


        self.gps_to_coords(lat, lat_direction, lon, lon_direction)

        if not self.use_gst:
            # Numbers below come from the GEM1305 spec.
            sigma_per_hdop = {
                4: 0.02,  # RTK fixed: ~2cm
                5: 0.20,  # RTK float: ~20cm
                2: 1.0,  # DGPS: ~1m
                1: 2.5,  # standalone: ~2.5m
                0: 100.0,  # no fix
            }.get(fix_quality, 5.0)

            sigma = hdop * sigma_per_hdop

            self.sigma_e = sigma
            self.sigma_n = sigma
            self.sigma_u = sigma
            self.gps_data_cov[0] = sigma ** 2  # sigma_x (east)
            self.gps_data_cov[7] = sigma ** 2  # sigma_y (north)
            self.gps_data_cov[14] = sigma ** 2  # sigma_z (vertical)


    def handle_gsa(self, fields):
        if not fields[16] or not fields[17]:
            return
        
        hdop = float(fields[16]) ** 2 if fields[16] else 0.0
        vdop = float(fields[17]) ** 2 if fields[17] else 0.0

        self.gps_data_cov[0] = hdop
        self.gps_data_cov[7] = hdop
        self.gps_data_cov[14] = vdop
        self.gps_data_cov[21] = 1e6
        self.gps_data_cov[28] = 1e6
        self.gps_data_cov[35] = 1e6

    def handle_gst(self, fields):
        """
        $GNGST,utc,rms,sigma_major,sigma_minor,orient_deg,sigma_lat,sigma_lon,sigma_alt*CS
        """
        sigma_n = float(fields[6]) if fields[6] else 0.0
        sigma_e = float(fields[7]) if fields[7] else 0.0
        sigma_u = float(fields[8]) if fields[8] else 0.0

        # Real GPS sigma is never >1km.
        if sigma_n <= 0.0 or sigma_e <= 0.0 or sigma_n > 1000.0 or sigma_e > 1000.0:
            return

        self.use_gst = True
        self.sigma_e = sigma_e
        self.sigma_n = sigma_n
        self.sigma_u = sigma_u
        self.gps_data_cov[0] = sigma_e ** 2  # sigma_x (east)
        self.gps_data_cov[7] = sigma_n ** 2  # sigma_y (north)
        self.gps_data_cov[14] = sigma_u ** 2  # sigma_z (up)

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

        self.lat = lat
        self.lon = lon

        #  An approx from earth global circ. to x, y
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

    def _rtcm_loop(self):
        while rclpy.ok():
            try:
                s = socket.create_connection(("localhost", 9195), timeout=5)
                self.logger.info("RTCM connected")
                while rclpy.ok():
                    data = s.recv(4096)
                    if not data:
                        break
                    self.ser.write(data)
                    self.rtcm_bytes_total += len(data)
                    self.rtcm_last_t = time.time()
            except Exception as e:
                self.logger.warning(f"RTCM: {e}, retry 2s")
                time.sleep(2)


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
