std_msgs/Header header

uint8 STATE_DISARMED=0
uint8 STATE_ARMED_IDLE=1
uint8 STATE_RUNNING=2
uint8 STATE_FAULT_LATCHED=3

uint8 state
bool motion_output_connected
bool candidate_seen
bool command_allowed
bool latched
string reason
geometry_msgs/Twist candidate
geometry_msgs/Twist supervised
