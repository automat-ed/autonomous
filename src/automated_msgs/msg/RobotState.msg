# Describes the state of the robot
uint8 robot_state
NetworkStatus network_status

# Defines the states of the robot
uint8 OFF = 0       # Software stack is not running
uint8 READY = 1     # Software stack is running, but waiting for start signal
uint8 DRIVING = 2   # Robot is moving
uint8 WAITING = 3   # Robot is waiting for some event
uint8 EMERGENCY = 4 # Robot has entered an emergency stop state
uint8 BLOCKED = 5   # Robot is blocked by an obstacle and cannot move