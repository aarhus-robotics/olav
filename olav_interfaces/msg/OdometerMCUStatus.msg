 #############################################################################
 #                            _     _     _     _                            #
 #                           / \   / \   / \   / \                           #
 #                          ( O ) ( L ) ( A ) ( V )                          #
 #                           \_/   \_/   \_/   \_/                           #
 #                                                                           #
 #                  OLAV: Off-Road Light Autonomous Vehicle                  #
 #############################################################################

# This message represents the current status of the Arduino Micro odometer
# microcontroller in OLAV.
#
# The message contains the readings obtained from the serial port of the
# microcontroller, which are in turn obtained from the vehicle CAN bus.

# Message header.
std_msgs/Header header

# The engine speed as shown on the vehicle digital dashboard, measured in
# revolutions per minute (rpm).
float64 engine_speed

# The average rear axle speed, measured in centidegrees per second, as read
# through the vehicle CAN bus.
float64 axle_speed

#
bool emergency_stop