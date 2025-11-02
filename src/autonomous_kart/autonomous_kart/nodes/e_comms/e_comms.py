
def pack_to_buffer(motor_percent: float, steering_angle: float) -> bytearray:
    """
    Pack the motor and steering commands into a bytearray buffer for SPI transmission.
    Caller is responsible for ensuring values are within expected ranges.
    
    :param motor_percent: Motor command as a percentage (0 to 100)
    :param steering_angle: Steering command as an angle in degrees (-90 to 90)
    :return: Bytearray buffer ready for SPI transmission of length 4.
             2 bytes for motor, 2 bytes for steering. Steering raw values are offset by +90
             to fit into unsigned int.
    """
    # Scale to raw values
    motor_raw_scaled = (2**16 - 1) * (motor_percent / 100.0) # 0-100 -> 0-(2^16-1)
    steering_raw_scaled = (2**16 - 1) * ((steering_angle + 90.0) / 180.0) # -90-90 -> 0-(2^16-1)

    # Create bytearray buffer
    buffer = bytearray(4)
    buffer[0] = (int(motor_raw_scaled) >> 8) & 0xFF  # Motor high byte
    buffer[1] = int(motor_raw_scaled) & 0xFF         # Motor low byte
    buffer[2] = (int(steering_raw_scaled) >> 8) & 0xFF  # Steering high byte
    buffer[3] = int(steering_raw_scaled) & 0xFF         # Steering low byte

    return buffer
