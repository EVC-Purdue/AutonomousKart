from typing import Optional

import traceback

import can
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, UInt16, Bool, String
from rclpy.impl.rcutils_logger import RcutilsLogger

import autonomous_kart.nodes.e_comms.e_comms as e_comms

CAN_CHANNEL = "/dev/ttyACM0"
CAN_BITRATE = 500000

CONTROL_ID = 0x100
STATUS_ID = 0x101
HEARTBEAT_ID = 0x102


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

        self.min_speed: float = float(self.get_parameter("min_speed").value)
        self.max_steering: float = float(self.get_parameter("max_steering").value)
        self.min_steering: float = float(self.get_parameter("min_steering").value)
        v_max_mps: float = float(self.get_parameter("v_max_mps").value)
        self.max_speed: float = v_max_mps
        self.pct_to_mps: float = v_max_mps / 100.0
        self.steer_max_deg: float = float(self.get_parameter("steer_max_deg").value)

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

        # Subscribe for combined [throttle, steering] commands. Both values must
        # arrive together — the CAN control frame carries both, so it never makes
        # sense to update one half on its own. Each message updates internal state
        # and immediately fires can_control_tx.
        self.drive_sub = self.create_subscription(Float32MultiArray, "cmd_drive", self.cmd_drive, 5)

        # CAN heartbeat
        self.hb_counter = 0
        self.hb_tx_period_ms = self.get_parameter("heartbeat_period_ms").value
        self.hb_timer = self.create_timer(self.hb_tx_period_ms / 1000.0, self.can_hb_tx)

        # Publishers
        self.adcb_state_pub = self.create_publisher(String, "e_comms/adcb_state", 1)
        self.rc_mode_pub = self.create_publisher(Bool, "e_comms/rc_mode", 1)
        self.throttle_pwm_pub = self.create_publisher(UInt16, "e_comms/throttle_pwm", 1)
        self.steering_pwm_pub = self.create_publisher(UInt16, "e_comms/steering_pwm", 1)

        # Init finished
        self.logger.info("Initialized EComms Node")

    # CAN RX ------------------------------------------------------------------#
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
    #--------------------------------------------------------------------------#

    # Command Callbacks -------------------------------------------------------#
    def cmd_drive(self, msg: Float32MultiArray):
        self.cmd_count += 1
        if len(msg.data) < 2:
            self.logger.error(f"cmd_drive payload too short: {list(msg.data)}")
            return
        self.steering_angle, self.throttle_percent = self.convert(
            float(msg.data[1]), float(msg.data[0]),
        )
        self.can_control_tx()  # Send updated command immediately on CAN bus

    def convert(self, steering: float, throttle: float) -> tuple:
        """Clamp + unit-convert (steering, throttle) for the CAN control frame."""
        if throttle < self.min_speed:
            throttle = self.min_speed
        elif throttle > self.max_speed:
            throttle = self.max_speed

        steering = (steering / self.steer_max_deg) * 100.0
        if steering < self.min_steering:
            steering = self.min_steering
        elif steering > self.max_steering:
            steering = self.max_steering
        return steering * 3, throttle * 1
    #--------------------------------------------------------------------------#

    # CAN TX ------------------------------------------------------------------#
    def can_hb_tx(self):
        """
        Send heartbeat message on CAN bus at a fixed rate.
        Also updates the heartbeat counter which is included in the message payload.
        """
        self.hb_counter = (self.hb_counter + 1) % 256  # Wrap around at 255
        hb_data = e_comms.pack_hb_message(self.hb_counter)

        if self.bus is not None:
            msg = can.Message(
                arbitration_id=HEARTBEAT_ID,
                data=hb_data,
                is_extended_id=False,
            )
            try:
                self.bus.send(msg)
            except can.CanError as e:
                self.logger.error(f"Failed to send CAN heartbeat message: {e}")

    def can_control_tx(self):
        """
        Send the latest throttle and steering commands on the CAN bus.
        Uses internal state updated by the command callbacks.
        """
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
    #--------------------------------------------------------------------------#

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
