"""
- Teeth on motor gear: 15
- Teeth on axle gear: 65
- Wheel diameter: 11 inches
- Pole pairs: 4
- RPM = ERPM / pole_pairs
- Speed(ERPM) in m/s =
	- Axle RPM * Wheel Circumference = 
	- (Motor RPM * gear_ratio) * (pi * wheel_diameter) =
	- (ERPM/pole_pairs * gear_ratio) * (pi * wheel_diameter) =
	- ((ERPM/pole_pairs) * (teeth_on_motor_gear/teeth_on_axle_gear) * (pi * wheel_diameter) =
	- ((ERPM/pole_pairs) * (teeth_on_motor_gear/teeth_on_axle_gear) * (pi * wheel_diameter_inches) * (1 meter / 39.37 inches) * (1 minute / 60 seconds)
"""
import math

TEETH_ON_MOTOR_GEAR = 15
TEETH_ON_AXLE_GEAR = 65
WHEEL_DIAMETER_INCHES = 11
POLE_PAIRS = 4
GEAR_RATIO = TEETH_ON_MOTOR_GEAR / TEETH_ON_AXLE_GEAR
WHEEL_CIRCUMFERENCE_INCHES = (math.pi * WHEEL_DIAMETER_INCHES)
INCHES_TO_METERS = 1 / 39.37
SECONDS_PER_MINUTE = 60.0

def erpm_to_speed(erpm: float) -> float:
	"""
	Convert ERPM (Electrical RPM) to linear speed in m/s.
	"""
	motor_rpm = erpm / POLE_PAIRS
	axle_rpm = motor_rpm * GEAR_RATIO
	wheel_circumference_meters = WHEEL_CIRCUMFERENCE_INCHES * INCHES_TO_METERS
	speed_m_per_s = (axle_rpm * wheel_circumference_meters) / SECONDS_PER_MINUTE
	return speed_m_per_s

def speed_to_erpm(speed_m_per_s: float) -> float:
	"""
	Convert linear speed in m/s to ERPM (Electrical RPM).
	"""
	wheel_circumference_meters = WHEEL_CIRCUMFERENCE_INCHES * INCHES_TO_METERS
	axle_rpm = (speed_m_per_s * SECONDS_PER_MINUTE) / wheel_circumference_meters
	motor_rpm = axle_rpm / GEAR_RATIO
	erpm = motor_rpm * POLE_PAIRS
	return erpm