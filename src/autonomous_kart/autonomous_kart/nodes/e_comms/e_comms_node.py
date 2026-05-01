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
            "e_comms_node",
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
        self.adcb_status: e_comms.AdcbStatus = e_comms.AdcbStatus(
            logic_mode="not initialized",
            rc_mode=False,
            throttle_pwm=0,
            steering_pwm=0,
        )

        # CAN bus
        if not self.simulation_mode:
            try:
                self.bus: Optional[can.interface.Bus] = can.interface.Bus(interface="slcan", channel=CAN_CHANNEL,
                                                                          bitrate=CAN_BITRATE)
                self.can_notifier: Optional[can.Notifier] = can.Notifier(self.bus, [self._on_can_msg])
                self.logger.info(f"Connected to CAN device on {CAN_CHANNEL} at {CAN_BITRATE} baud")
            except FileNotFoundError:
                self.logger.error(f"Can device not connected on {CAN_CHANNEL}")
                self.simulation_mode = True  # Running on-device, off-kart

                self.bus: Optional[can.interface.Bus] = None
                self.can_notifier: Optional[can.Notifier] = None
        else:
            self.bus: Optional[can.interface.Bus] = None
            self.can_notifier: Optional[can.Notifier] = None
            self.logger.info("Running in simulation mode, CAN bus disabled")

        # Logging
        self.cmd_count: int = 0
        self.last_log_time: Time = self.get_clock().now()

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscribe to pathfinder node for throttle and steering commands
        # self.throttle_sub: Subscriber = Subscriber(self, Float32, "cmd_vel")
        # self.steering_sub: Subscriber = Subscriber(self, Float32, "cmd_turn")
        # self.ts: ApproximateTimeSynchronizer = ApproximateTimeSynchronizer(
        #     [self.throttle_sub, self.steering_sub],
        #     queue_size=3,
        #     slop=0.01,  # Time tolerance in seconds
        #     allow_headerless=True,  # Float32 has no header
        # )
        # self.ts.registerCallback(self.cmd_callback)
        self.steering_sub = self.create_subscription(Float32, "cmd_turn", self.cmd_steer, 5)
        self.throttle_sub = self.create_subscription(Float32, "cmd_vel", self.cmd_thr, 5)

        # Publishers
        self.adcb_state_pub = self.create_publisher(
            String, "e_comms/adcb_state", 1
        )
        self.rc_mode_pub = self.create_publisher(
            Bool, "e_comms/rc_mode", 1
        )
        self.throttle_pwm_pub = self.create_publisher(
            UInt16, "e_comms/throttle_pwm", 1
        )
        self.steering_pwm_pub = self.create_publisher(
            UInt16, "e_comms/steering_pwm", 1
        )

        # Init finished
        self.logger.info("Initialize EComms Node")

    def cmd_thr(self, throttle_msg: Float32):
        """
        Send the throttle and steering command to the Electrical Stack via CAN.
        Part of hot loop so must be efficient

        :param throttle_msg: Float32 message for throttle command (percent throttle)
        :param steering_msg: Float32 message for steering command (percent steering, -100 to 100)
        :return: None
        """
        self.cmd_count += 1

        # Ensure values are within expected ranges
        if not (0.0 <= throttle_msg.data <= 100.0):
            self.logger.error(
                f"Received out-of-bounds command values: "
                f"throttle_percent={throttle_msg.data}, steering_angle={self.steering_angle}"
            )
            raise ValueError("Command values out of bounds")

        self.throttle_percent = throttle_msg.data
        tx_data = e_comms.pack_control_message(self.throttle_percent, self.steering_angle)

        if self.bus is not None:
            msg = can.Message(
                arbitration_id=CONTROL_ID,
                data=tx_data,
                is_extended_id=False,
            )
            try:
                self.bus.send(msg)
                self.logger.info(f"Sent CAN: {msg.data.hex()}")
            except can.CanError as e:
                self.logger.error(f"Failed to send CAN message: {e}")


    def cmd_steer(self, steering_msg: Float32):
        """
        Send the throttle and steering command to the Electrical Stack via CAN.
        Part of hot loop so must be efficient

        :param throttle_msg: Float32 message for throttle command (percent throttle)
        :param steering_msg: Float32 message for steering command (percent steering, -100 to 100)
        :return: None
        """
        self.cmd_count += 1

        # Ensure values are within expected ranges
        if not (
                -100.0 <= steering_msg.data <= 100.0
        ):
            self.logger.error(
                f"Received out-of-bounds command values: "
                f"throttle_percent={self.throttle_percent}, steering_angle={steering_msg.data}"
            )
            raise ValueError("Command values out of bounds")

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

    def _on_can_msg(self, msg: can.Message):
        """Called by can.Notifier in a background thread for each received message."""
        if msg.arbitration_id == STATUS_ID:
            data = bytes(msg.data)  # copy, don't hold a reference
            # Add to executor to handle in main thread because ros publishers are not thread safe
            if self.executor is not None:
                self.executor.create_task(lambda: self.handle_status_msg(data))
            else:
                self.logger.warning("CAN message received before executor ready, dropping frame")

    def handle_status_msg(self, msg_data: bytes):
        try:
            self.adcb_status = e_comms.unpack_status_message(msg_data, self.logger)

            self.adcb_state_pub.publish(String(data=self.adcb_status.logic_mode))
            self.rc_mode_pub.publish(Bool(data=self.adcb_status.rc_mode))
            self.throttle_pwm_pub.publish(UInt16(data=self.adcb_status.throttle_pwm))
            self.steering_pwm_pub.publish(UInt16(data=self.adcb_status.steering_pwm))
        except Exception as e:
            self.logger.error(f"Failed to parse CAN message: {e}")

    def destroy_node(self):
        # Close CAN
        if self.can_notifier is not None:
            self.can_notifier.stop()
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
