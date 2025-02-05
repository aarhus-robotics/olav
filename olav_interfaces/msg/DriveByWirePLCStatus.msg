 #############################################################################
 #                            _     _     _     _                            #
 #                           / \   / \   / \   / \                           #
 #                          ( O ) ( L ) ( A ) ( V )                          #
 #                           \_/   \_/   \_/   \_/                           #
 #                                                                           #
 #                  OLAV: Off-Road Light Autonomous Vehicle                  #
 #############################################################################

# This message represents the current status of the Wago PLC controlling the
# drive-by-wire system in OLAV. The message contains the register values
# written by the user to control the drive-by-wire system (prefixed by
# "setpoint_"), as well as the register values read from the PLC and detailing
# its current state (prefixed by "feedback_").

# Message header.
std_msgs/Header header

# Value written to PLC register at address 0x302, representing the current
# desired throttle effort in the range 0 - 100, corresponding to fully released
# and fully depressed throttle pedal, respectively.
int16 setpoint_throttle

# Value written to PLC register at address 0x301, representing the current
# desired brake effort in the range 0 - 100, corresponding to fully released
# and fully depressed brake pedal, respectively.
int16 setpoint_brake

# Value written to PLC register at address 0x300, representing the current
# desired steering effort in the range -6000 - 6000.
int16 setpoint_steering

# Value written to PLC register at address 0x307, representing the current
# desired gear as an integer value mapped to the following enumeration type:
# 1 - PARK
# 2 - REVERSE
# 3 - NEUTRAL
# 4 - LOW
# 5 - HIGH
int16 setpoint_gear

# Value written to PLC register at address 0x303, representing the state of the
# ignition switch (true if ignition is on, false if ignition is off). Note that
# this must be the negation of the PLC register value at address 0x304.
bool setpoint_ignition_on

# Value written to PLC register at address 0x304, representing the state of the
# ignition switch (false if ignition is on, true if ignition is off). Note that
# this must be the negation of the PLC register value at address 0x303.
bool setpoint_ignition_off

# Value written to PLC register at address 0x306, representing the state of the
# engine starter switch.
bool setpoint_engine_starter

# Value written to PLC register at address 0x305, representing the state of the
# emergency stop switch.
bool setpoint_emergency_stop

# Value read from PLC register at address 0x200, representing the current
# steering acuator transducer reading in millivolts.
int16 steering_actuator_position_raw

# Result of the conversion of the steering actuator transducer reading from
# millivolts to radians using the internal calibration curve.
float64 steering_wheel_angle_radians

# Result of the conversion of the steering actuator transducer reading from
# millivolts to radians using the internal calibration curve, converted to
# degrees.
float64 steering_wheel_angle_degrees

# Value read from PLC register at addresss 0x201, representing the currently
# selected gear as an integer value mapped to the following enumeration type:
# 1 - PARK
# 2 - REVERSE
# 3 - NEUTRAL
# 4 - LOW
# 5 - HIGH
int16 selected_gear

# Value read from PLC register at addresss 0x202, representing the current
# position of the gear actuator.
int16 gear_actuator_position

# Value read from PLC register at addresss 0x203, representing the current
# position of the brake actuator.
int16 brake_actuator_position

# Value read from PLC register at addresss 0x204, representing the current
# ignition status.
bool is_ignition_on

# Value read from PLC register at addresss 0x205, representing the current state
# of the autonomous mode switch.
bool is_autonomous_mode_on

# Value read from PLC register at address 0x206, representing whether the gear
# actuator has reached the position matching the currently selected gear.
bool is_gear_actuator_in_position