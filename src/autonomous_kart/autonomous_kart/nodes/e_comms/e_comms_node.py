from typing import Optional

import traceback

import can
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, UInt16, Bool, String
from rclpy.impl.rcutils_logger import RcutilsLogger
from message_filters import Subscriber, ApproximateTimeSynchronizer

import autonomous_kart.nodes.e_comms.e_comms as e_comms

CAN_CHANNEL = "/dev/ttyACM0"
CAN_BITRATE = 500000

CONTROL_ID = 0x100
STATUS_ID = 0x101


class ECommsNode(Node):
    def __init__(self):
        super().__init__(
            "ECommsNode",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger: RcutilsLogger = self.get_logger()

        # Parameters
        self.simulation_mode: bool = self.get_parameter("simulation_mode").value

        # Inputs
        self.throttle_percent: float = 0.0
        self.steering_angle: float = 0.0

        # Outputs
        self.adb_state: str = "not initialized"
        self.rc_mode: bool = False
        self.throttle_pwm: int = 0
        self.steering_pwm: int = 0

        # CAN bus
        if not self.simulation_mode:
            try:
                self.bus: Optional[can.interface.Bus] = can.interface.Bus(interface="slcan", channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
            except FileNotFoundError:
                self.logger.error("Can device not connected on {CAN_CHANNEL}")
                self.simulation_mode = True  # Running on-device, off-kart
                self.bus = None
        else:
            self.bus: Optional[can.interface.Bus] = None

        # Timer to poll CAN RX
        self.create_timer(0.5, self.poll_can)

        # Logging
        self.cmd_count: int = 0
        self.last_log_time: Time = self.get_clock().now()

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscribe to pathfinder node for throttle and steering commands
        self.throttle_sub: Subscriber = Subscriber(self, Float32, "cmd_vel")
        self.steering_sub: Subscriber = Subscriber(self, Float32, "cmd_turn")
        self.ts: ApproximateTimeSynchronizer = ApproximateTimeSynchronizer(
            [self.throttle_sub, self.steering_sub],
            queue_size=3,
            slop=0.1,  # Time tolerance in seconds
            allow_headerless=True,  # Float32 has no header
        )
        self.ts.registerCallback(self.cmd_callback)

        # Publishers
        self.adb_state_pub = self.create_publisher(
            String, "e_comms/rx/adb_state", 1
        )
        self.rc_mode_pub = self.create_publisher(
            Bool, "e_comms/rx/rc_mode", 1
        )
        self.throttle_pwm_pub = self.create_publisher(
            UInt16, "e_comms/rx/throttle", 1
        )
        self.steering_pwm_pub = self.create_publisher(
            UInt16, "e_comms/rx/steering", 1
        )

        # Init finished
        self.logger.info("Initialize EComms Node")

    def cmd_callback(self, throttle_msg: Float32, steering_msg: Float32):
        """
        Send the throttle and steering command to the Electrical Stack via CAN.
        Part of hot loop so must be efficient

        :param throttle_msg: Float32 message for throttle command (percent throttle)
        :param steering_msg: Float32 message for steering command (percent steering, -100 to 100)
        :return: None
        """
        self.cmd_count += 1

        # Ensure values are within expected ranges
        if not (0.0 <= throttle_msg.data <= 100.0) or not (
            -100.0 <= steering_msg.data <= 100.0
        ):
            self.logger.error(
                f"Received out-of-bounds command values: "
                f"throttle_percent={throttle_msg.data}, steering_angle={steering_msg.data}"
            )
            raise ValueError("Command values out of bounds")

        self.throttle_percent = throttle_msg.data
        self.steering_angle = steering_msg.data
        tx_data = e_comms.pack_control_message(self.throttle_percent, self.steering_angle)

        if self.bus is not None:
            msg = can.Message(
                arbitration_id=CONTROL_ID,
                data=tx_data,
                is_extended_id=False,
            )
            try:
                self.bus.send(msg)
            except can.CanError as e:
                self.logger.error(f"Failed to send CAN message: {e}")
    
    def poll_can(self):
        if self.bus is not None:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return

            if msg.arbitration_id == STATUS_ID:
                try:
                    adb_state, rc_mode, throttle_pwm, steering_pwm = e_comms.unpack_status_message(msg.data, self.logger)

                    self.adb_state = adb_state
                    self.rc_mode = rc_mode
                    self.throttle_pwm = throttle_pwm
                    self.steering_pwm = steering_pwm

                    self.adb_state_pub.publish(String(data=self.adb_state))
                    self.rc_mode_pub.publish(Bool(data=self.rc_mode))
                    self.throttle_pwm_pub.publish(UInt16(data=self.throttle_pwm))
                    self.steering_pwm_pub.publish(UInt16(data=self.steering_pwm))
                except Exception as e:
                    self.logger.error(f"Failed to parse CAN message: {e}")

    def destroy_node(self):
        # Close CAN
        if self.bus is not None:
            self.bus.shutdown()

        # Rest of node teardown
        super().destroy_node()

    def log_command_rate(self):
        """Log average commands per second every 5 seconds"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_log_time).nanoseconds / 1e9

        if elapsed > 0:
            avg_rate = self.cmd_count / elapsed
            self.logger.info(
                f"Average command rate: {avg_rate:.2f} commands/sec "
                f"(Total: {self.cmd_count} in {elapsed:.1f}s)"
            )

        # Reset counters
        self.cmd_count = 0
        self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)

    node = ECommsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Context already shutdown, ignore


if __name__ == "__main__":
    main()
