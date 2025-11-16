import struct

from rclpy.impl.rcutils_logger import RcutilsLogger


def pack_to_tx_buffer(motor_percent: float, steering_angle: float) -> list[int]:
    """
    Pack the motor and steering commands into a bytearray buffer for SPI transmission.
    Caller is responsible for ensuring values are within expected ranges.
    
    :param motor_percent: Motor command as a percentage (0 to 100)
    :param steering_angle: Steering command as an angle in degrees (-90 to 90)
    :return: List (u8 buffer) ready for SPI transmission of length 4.
             2 bytes for motor, 2 bytes for steering. Steering raw values are offset by +90
             to fit into unsigned int.
    """
    # Scale to raw values
    motor_raw_scaled = (2**16 - 1) * (motor_percent / 100.0) # 0-100 -> 0-(2^16-1)
    steering_raw_scaled = (2**16 - 1) * ((steering_angle + 90.0) / 180.0) # -90-90 -> 0-(2^16-1)

    # Create bytearray buffer
    buffer = [0]*4
    buffer[0] = (int(motor_raw_scaled) >> 8) & 0xFF     # Motor high byte
    buffer[1] = int(motor_raw_scaled) & 0xFF            # Motor low byte
    buffer[2] = (int(steering_raw_scaled) >> 8) & 0xFF  # Steering high byte
    buffer[3] = int(steering_raw_scaled) & 0xFF         # Steering low byte

    return buffer

def unpack_from_rx_buffer(rx_buffer: list[int], logger: RcutilsLogger) -> float:
    """
    Unpack the motor feedback from the received SPI buffer.
    
    :param rx_buffer: List (u8 buffer) received from SPI transmission of length 4.
                      4 bytes of a raw float representing motor RPM.
    :return: Motor RPM as float. Raises ValueError if buffer length is not 4.
    """
    if len(rx_buffer) != 4:
        logger.error(f"RX buffer length invalid: expected 4, got {len(rx_buffer)}")
        raise ValueError(f"RX buffer must be of length 4, got {len(rx_buffer)}")
    
    rx_bytes = bytearray(rx_buffer)
    motor_rpm = struct.unpack('>f', rx_bytes)[0]
    return motor_rpm