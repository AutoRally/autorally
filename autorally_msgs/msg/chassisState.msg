uint_8 CONTROL_MODE_MANUAL = 0
uint_8 CONTROL_MODE_AUTONOMOUS = 1

uint_8 THROTTLE_RELAY_DISABLED=0
uint_8 THROTTLE_RELAY_ENABLED=1

Header header

uint_8 controlMode
uint_8 throttleRelay
autorally_msgs/runstop runstop

string steeringCommander
float64 steering

string throttleCommander
float64 throttle

string frontBrakeCommander
float64 frontBrake