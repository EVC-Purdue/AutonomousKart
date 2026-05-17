from typing import Optional

import traceback

import can
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, UInt16, String, Float32
from rclpy.impl.rcutils_logger import RcutilsLogger

import autonomous_kart.nodes.e_comms.e_comms as e_comms
import autonomous_kart.nodes.e_comms.powertrain as powertrain

CAN_CHANNEL = "/dev/ttyACM0"
CAN_BITRATE = 500000

VESC_ID = 7
VESC_STATUS_1_MSG_NUM = 9
VESC_MSG_NUM_TO_EXT_ID = lambda msg_num: ((msg_num << 8) | VESC_ID)

CONTROL_ID = 0x100
STATUS_ID = 0x101
HEARTBEAT_ID = 0x102
VESC_STATUS_1_ID = VESC_MSG_NUM_TO_EXT_ID(VESC_STATUS_1_MSG_NUM)


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
        self.throttle_erpm: float = 0.0
        self.steering_percent: float = 0.0

        # Outputs
        self.adcb_status: e_comms.AdcbStatus = e_comms.AdcbStatus(
            logic_state="not initialized",
            running_mode="not initialized",
            throttle_pwm=0,
            steering_pwm=0,
        )
        self.vesc_status_1: e_comms.VescCanStatus1 = e_comms.VescCanStatus1(
            erpm=0,
            current=0,
            duty_cycle=0,
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
        self.logic_state_pub = self.create_publisher(String, "e_comms/logic_state", 1)
        self.running_mode_pub = self.create_publisher(String, "e_comms/running_mode", 1)
        self.throttle_pwm_pub = self.create_publisher(UInt16, "e_comms/throttle_pwm", 1)
        self.steering_pwm_pub = self.create_publisher(UInt16, "e_comms/steering_pwm", 1)
        self.speed_pub = self.create_publisher(Float32, "e_comms/kart_speed_m_per_s", 1)

        # Init finished
        self.logger.info("Initialized EComms Node")

    # CAN RX ------------------------------------------------------------------#
    def _on_can_msg(self, msg: can.Message):
        """Called by can.Notifier in a background thread for each received message."""
        if self.executor is None:
            self.logger.warning("CAN message received before executor ready, dropping frame")
            return
        
        data = bytes(msg.data)  # copy, don't hold a reference
        self.cmd_count += 1
        # self.logger.info(f"Msg: {msg.arbitration_id:X}")
        if msg.arbitration_id == STATUS_ID:
            self.executor.create_task(lambda: self.handle_adcb_status_msg(data))
        elif msg.arbitration_id == VESC_STATUS_1_ID:
            self.executor.create_task(lambda: self.handle_vesc_status_1_msg(data))

    def handle_adcb_status_msg(self, msg_data: bytes):
        try:
            self.adcb_status = e_comms.unpack_adcb_status_message(msg_data, self.logger)

            self.logic_state_pub.publish(String(data=self.adcb_status.logic_state))
            self.running_mode_pub.publish(String(data=self.adcb_status.running_mode))
            self.throttle_pwm_pub.publish(UInt16(data=self.adcb_status.throttle_pwm))
            self.steering_pwm_pub.publish(UInt16(data=self.adcb_status.steering_pwm))
        except Exception as e:
            self.logger.error(f"Failed to parse CAN message: {e}")

    def handle_vesc_status_1_msg(self, msg_data: bytes):
        try:
            self.vesc_status_1 = e_comms.unpack_vesc_status_1_message(msg_data, self.logger)
            speed_m_per_s = powertrain.erpm_to_speed(self.vesc_status_1.erpm)
            self.speed_pub.publish(Float32(data=speed_m_per_s))
        except Exception as e:
            self.logger.error(f"Failed to parse VESC status message: {e}")
    #--------------------------------------------------------------------------#

    # Command Callbacks -------------------------------------------------------#
    def cmd_drive(self, msg: Float32MultiArray):
        self.cmd_count += 1
        if len(msg.data) < 2:
            self.logger.error(f"cmd_drive payload too short: {list(msg.data)}")
            return
        throttle_cmd = msg.data[0] # m/s
        steering_cmd = msg.data[1] # degrees (negative = left, positive = right)
        self.throttle_erpm, self.steering_percent = self.convert(throttle_cmd, steering_cmd)
        self.can_control_tx()  # Send updated command immediately on CAN bus

    def convert(self, throttle_m_per_s: float, steering_deg: float) -> tuple[float, float]:
        """Clamp + unit-convert (steering, throttle) for the CAN control frame."""
        steering_deg *= 1
        clamped_throttle_m_per_s = max(self.min_speed, min(self.max_speed, throttle_m_per_s))
        throttle_erpm = powertrain.speed_to_erpm(clamped_throttle_m_per_s)

        clamped_steering_deg = max(self.min_steering, min(self.steer_max_deg, steering_deg))
        max_one_direction_steering = max(abs(self.min_steering), abs(self.steer_max_deg))
        steering_percent = (clamped_steering_deg / max_one_direction_steering) * 100.0
        return throttle_erpm, steering_percent
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
        tx_data = e_comms.pack_control_message(self.throttle_erpm, self.steering_percent)
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

