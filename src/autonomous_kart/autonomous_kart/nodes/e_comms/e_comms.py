from rclpy.impl.rcutils_logger import RcutilsLogger


def pack_to_tx_buffer(motor_percent: float, steering_angle: float) -> list[int]:
    """
    Pack the motor and steering commands into a bytearray buffer for SPI transmission.
    Caller is responsible for ensuring values are within expected ranges.
    
    :param motor_percent: Motor command as a percentage (0 to 100)
    :param steering_angle: Steering command as percent steering (-100 to 100, 0 = center)
    :return: List (u8 buffer) ready for SPI transmission of length 4.
             2 bytes for motor, 2 bytes for steering. Steering raw values are offset by +90
             to fit into unsigned int.
    """
    # Scale to raw values

    motor_raw_scaled = (2**16 - 1) * (motor_percent / 100.0) # 0-100 -> 0-(2^16-1)
    steering_raw_scaled = (2**16 - 1) * ((steering_angle + 100.0) / 200.0) # -100-100 -> 0-(2^16-1)

    # Create bytearray buffer
    buffer = [0]*4
    buffer[0] = (int(motor_raw_scaled) >> 8) & 0xFF     # Motor high byte
    buffer[1] = int(motor_raw_scaled) & 0xFF            # Motor low byte
    buffer[2] = (int(steering_raw_scaled) >> 8) & 0xFF  # Steering high byte
    buffer[3] = int(steering_raw_scaled) & 0xFF         # Steering low byte

    return buffer

def unpack_from_rx_buffer(rx_buffer: list[int], logger: RcutilsLogger) -> tuple[int, int]:
    """
    Unpack the motor feedback from the received SPI buffer.
    
    :param rx_buffer: List (u8 buffer) received from SPI transmission of length 4.
                      2 bytes for the motor PWM pulse, 2 bytes for steering.
    :return: A tuple containing the motor PWM pulse (int) and steering PWM (int).
    """
    if len(rx_buffer) != 4:
        logger.error(f"RX buffer length invalid: expected 4, got {len(rx_buffer)}")
        raise ValueError(f"RX buffer must be of length 4, got {len(rx_buffer)}")
    
    motor_pwm = (rx_buffer[0] << 8) | rx_buffer[1]
    steering_pwm = (rx_buffer[2] << 8) | rx_buffer[3]
    return motor_pwm, steering_pwm
    
    