from dataclasses import dataclass
from rclpy.impl.rcutils_logger import RcutilsLogger

# typedef enum {
# 	LOGIC_MODE_STARTING = 0,
# 	LOGIC_MODE_PRECHARGING,
# 	LOGIC_MODE_CONTACTOR_CLOSING,
# 	LOGIC_MODE_RUNNING,
# 	LOGIC_MODE_ESTOPPED,
# 	LOGIC_MODE_RC_DISCONNECTED,
# 	LOGIC_MODE_CAN_DISCONNECTED,
# 	LOGIC_MODE_RECOVERING,
# } logic_mode_t;

MODE_VALUE_TO_STRING = {
    0: "STARTING",
    1: "PRECHARGING",
    2: "CONTACTOR_CLOSING",
    3: "RUNNING",
    4: "ESTOPPED",
    5: "RC_DISCONNECTED",
    6: "CAN_DISCONNECTED",
    7: "RECOVERING",
}

@dataclass
class AdcbStatus:
    logic_mode: str   # String representation of the state machine mode (e.g., "RUNNING")
    rc_mode: bool     # Boolean indicating RC mode (True if in autonomous mode, False if in RC mode)
    throttle_pwm: int # Integer value of the throttle PWM (1000-2000)
    steering_pwm: int # Integer value of the steering PWM (1000-2000)

@dataclass
class VescCanStatus1:
    erpm: int         # ERPM of the motor
    current: int      # Motor current in 0.1A (e.g., 100 = 10A)
    duty_cycle: int   # Duty cycle in 0.001 (e.g., 1000 = 100%)


def pack_control_message(throttle_erpm: int, steering_percent: float) -> bytes:
    """
    Pack the throttle and steering commands into a bytearray buffer for CAN
    Caller is responsible for ensuring values are within expected ranges.

    - ID = 0x100 - **Control commands**
        - Byte 0-1: throttle ERPM (uint16_t, little endian), 0-AUTONOMOUS_ERPM_MAX (0 = full stop, AUTONOMOUS_ERPM_MAX = max speed)
        - Byte 2-3: steering representation (uint16_t, little endian), 0-1000, where 500 = straight, 0 = full left, 1000 = full right
        - Byte 4-7: reserved / future use
        
    :param throttle_percent: throttle command as a percentage (0 to 100)
    :param steering_angle: Steering command as percent steering (-100 to 100, 0 = center)
    :return: byte array of length 8.
    """
    steering = int(500 + 500 * (steering_percent / 100.0))  # -100..100 -> 0..1000

    data = bytearray(8)
    data[0] = throttle_erpm & 0xFF
    data[1] = (throttle_erpm >> 8) & 0xFF

    data[2] = steering & 0xFF
    data[3] = (steering >> 8) & 0xFF

    # bytes 4-7 reserved for future use, set to 0
    data[4] = 0
    data[5] = 0
    data[6] = 0
    data[7] = 0

    return bytes(data)


def pack_hb_message(counter: int) -> bytes:
    """
    Pack a heartbeat message with a simple counter.

    - ID = `0x102` - **E_Comms heartbeat** (RX)
        - Byte 0: E_Comms heartbeat counter (uint8_t)
        - Byte 1-7: reserved / future use

    :param counter: Heartbeat counter (does not have to be 0-255, will be truncated to uint8_t)
    :return: byte array of length 8.
    """
    data = bytearray(8)
    data[0] = counter & 0xFF  # Truncate to uint8_t

    # bytes 1-7 reserved for future use, set to 0
    for i in range(1, 8):
        data[i] = 0

    return bytes(data)


def unpack_adcb_status_message(data: bytes, logger: RcutilsLogger) -> AdcbStatus:
    """
    Unpack the Autonomous Distro/Control Board (ADCB) status message received from the CAN bus.

    - ID = 0x101 - **Status update** (TX)
        - Byte 0: state machine mode + rc mode
            - Bits0-3 = state machine mode (see logic.h::logic_mode_t)
            - Bit4 = RC mode (0 = rc mode, 1 = autonomous mode)
            - Bits5-7 = reserved
        - Byte 1-2: throttle PWM (uint16_t, little endian), the actual PWM value being sent to the ESC for throttle (1000-2000)
        - Byte 3-4: steering PWM (uint16_t, little endian), the actual PWM value being sent to the servo for steering (1000-2000)
        - Byte 5-7: reserved / future use
    
    :param data: byte array of length 8 (>= 5) received from CAN message
    :param logger: Logger for logging errors
    :return: An AdcbStatus object containing the unpacked data
    """
    if len(data) < 5:
        logger.error(f"CAN RX data length invalid: expected 8 (or >=5), got {len(data)}")
        raise ValueError(f"CAN RX data length invalid: expected 8 (or >=5), got {len(data)}")
    
    logic_mode = data[0] & 0x0F  # Bits 0-3
    rc_mode = (data[0] & 0x10) >> 4 # Bit 4
    throttle_pwm = int(data[1]) | (int(data[2]) << 8)
    steering_pwm = int(data[3]) | (int(data[4]) << 8)

    logic_mode_str = MODE_VALUE_TO_STRING.get(logic_mode, f"UNKNOWN({logic_mode})")
    rc_mode_bool = bool(rc_mode)

    return AdcbStatus(logic_mode_str, rc_mode_bool, throttle_pwm, steering_pwm)

def unpack_vesc_status_1_message(data: bytes, logger: RcutilsLogger) -> VescCanStatus1:
    """
    Unpack the VESC status message 1 received from the CAN bus.

    - ID = `CAN_VESC_MSG_NUM_TO_EXT_ID(9 = CAN_VESC_STATUS_1_MSG_NUM)` (ext id) - **VESC status 1** (RX)
        - Byte 0-3: VESC ERPM (BE)
        - Byte 4-5: VESC current (in 0.1A, so 100 = 10A) (BE)
        - Byte 6-7: VESC duty cycle (in 0.001, so 1000 = 100%) (BE)

    :param data: byte array of length 8 received from CAN message
    :param logger: Logger for logging errors
    :return: A VescCanStatus1 object containing the unpacked data
    """
    if len(data) != 8:
        logger.error(f"VESC status message data length invalid: expected 8, got {len(data)}")
        raise ValueError(f"VESC status message data length invalid: expected 8, got {len(data)}")
    
    erpm = int.from_bytes(data[0:4], byteorder="big", signed=True)
    current = int.from_bytes(data[4:6], byteorder="big", signed=True)
    duty_cycle = int.from_bytes(data[6:8], byteorder="big", signed=True)

    return VescCanStatus1(erpm, current, duty_cycle)
