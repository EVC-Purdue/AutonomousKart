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
