from rclpy.impl.rcutils_logger import RcutilsLogger

# typedef enum {
# 	LOGIC_MODE_STARTING = 0,
# 	LOGIC_MODE_PRECHARGING,
# 	LOGIC_MODE_CONTACTOR_CLOSING,
# 	LOGIC_MODE_RUNNING,
# 	LOGIC_MODE_ESTOPPED,
# 	LOGIC_MODE_RC_DISCONNECTED,
# 	LOGIC_MODE_CAN_DISCONNECTED
# } logic_mode_t;

MODE_VALUE_TO_STRING = {
    0: "STARTING",
    1: "PRECHARGING",
    2: "CONTACTOR_CLOSING",
    3: "RUNNING",
    4: "ESTOPPED",
    5: "RC_DISCONNECTED",
    6: "CAN_DISCONNECTED"
}


def pack_control_message(throttle_percent: float, steering_angle: float) -> bytes:
    """
    Pack the throttle and steering commands into a bytearray buffer for CAN
    Caller is responsible for ensuring values are within expected ranges.

    - ID = 0x100 - **Control commands**
        - Byte 0-1: throttle (uint16_t, little endian), 0-1000, where 1000 = full throttle
        - Byte 2-3: steering (uint16_t, little endian), 0-1000, where 500 = straight, 0 = full left, 1000 = full right
        - Byte 4-7: reserved / future use
        
    :param throttle_percent: throttle command as a percentage (0 to 100)
    :param steering_angle: Steering command as percent steering (-100 to 100, 0 = center)
    :return: byte array of length 8.
    """
    throttle = int(1000 * (throttle_percent / 100.0))  # 0-1000
    steering = int(500 + 500 * (steering_angle / 100.0))  # -100..100 -> 0..1000

    data = bytearray(8)
    data[0] = throttle & 0xFF
    data[1] = (throttle >> 8) & 0xFF

    data[2] = steering & 0xFF
    data[3] = (steering >> 8) & 0xFF

    # bytes 4-7 reserved for future use, set to 0
    data[4] = 0
    data[5] = 0
    data[6] = 0
    data[7] = 0

    return bytes(data)

def unpack_status_message(data: bytes, logger: RcutilsLogger) -> tuple[str, bool, int, int]:
    """
    Unpack the throttle feedback from the received CAN buffer.

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
    :return: A tuple containing the unpacked data:
            - state_mode: String representation of the state machine mode (e.g., "RUNNING")
            - rc_mode: Boolean indicating RC mode (True if in autonomous mode, False if in RC mode)
            - throttle_pwm: Integer value of the throttle PWM (1000-2000)
            - steering_pwm: Integer value of the steering PWM (1000-2000)
    """
    if len(data) < 5:
        logger.error(f"CAN RX data length invalid: expected 8 (or >=5), got {len(data)}")
        raise ValueError(f"CAN RX data length invalid: expected 8 (or >=5), got {len(data)}")
    
    state_mode = data[0] & 0x0F  # Bits 0-3
    rc_mode = (data[0] & 0x10) >> 4 # Bit 4
    throttle_pwm = int(data[1]) | (int(data[2]) << 8)
    steering_pwm = int(data[3]) | (int(data[4]) << 8)

    state_str = MODE_VALUE_TO_STRING.get(state_mode, f"UNKNOWN({state_mode})")
    rc_mode_bool = bool(rc_mode)

    return state_str, rc_mode_bool, throttle_pwm, steering_pwm
    
    