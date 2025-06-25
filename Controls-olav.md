| Control | Description | Available in | Action | Action (LMOD) | Action (RMOD) | Feedback |
| --- | --- | --- | --- | --- | --- | --- |
| Left stick horizontal | Analog steering controls | CONTROL_TRIGGER_TBS | Sets the steering angle | - | - | No |  
| Left trigger | Analog brake controls | CONTROL_TRIGGER_TBS | Sets the brake effort | - | - | No |
| Right trigger | Analog throttle controls | CONTROL_TRIGGER_TBS | Sets the throttle effort | - | - | No |
| Pad horizontal | Schedules the horizontal trim | CONTROL_PAD_TBS, CONTROL_PAD_DRIVE | Schedules the horizontal trim | - | 10x multiplier | No |
| Pad vertical | Schedules the vertical trim | CONTROL_PAD_TBS, CONTROL_PAD_DRIVE | Schedules the vertical trim | - | 10x multiplier | No |
| View button | Cycle the ignition state | ALL | Send cycle request | - | - | Yes |
| Xbox button | Send ready to run request | ALL | Send request | - | - | Yes |
| Menu button | Start engine | ALL | Send request | - | - | Yes |
| Share button | Set mode | ? | Set mode to CONTROL_TRIGGER_TBS | Set mode to CONTROL_PAD_TBS | Set mode to CONTROL_PAD_DRIVE | Yes |
| Left bumper | Downshift | ALL | Send request | - | - | Yes |
| Right bumper | Upshift | ALL | Send request | - | - | Yes |
| X button | Reset trims | ALL | Reset trims state | - | - | Yes |
| Y button | Control datalogger | ALL | Start datalogger | Stop datalogger | - | Yes |
| A button | Apply trims | ALL | Apply trims state | - | - | Yes |
| B button | Emergency stop | ALL | Send request | - | - | Yes |

* Gamepad modes:
    * STANDBY mode: null throttle, null brake and null steering command published to drive-by-wire.
    * CONTROL_TRIGGER_TBS: brake set by left trigger, throttle set by right trigger, always applied
    * CONTROL_PAD_TBS: throttle/brake set by vertical pad, steering angle set by horizontal pad, apply with A
    * CONTROL_PAD_DRIVE: speed set by vertical pad, steering angle set by horizontal pad, apply with A
    *

* Control modes:
    * STANDBY: No controls are passed to the PLC
    * DRIVE_ACKERMANN: Ackermann commands are parsed to the speed and steering controllers, the outputs are passed to the PLC
    * DRIVE_TBS: raw throttle, brake and steering commands are passed to the PLC

* Authorities:
    * System: no controls muxed, controls from drive-by-wire internal code are still executed
    * Terminal: controls from terminal are muxed
    * Gamepad: controls from the gamepad are muxed
    * Autonomy: controls from an autonomy stack are muxed