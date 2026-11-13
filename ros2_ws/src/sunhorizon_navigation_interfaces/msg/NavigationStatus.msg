std_msgs/Header header

uint8 STATE_UNCONFIGURED=0
uint8 STATE_IDLE=1
uint8 STATE_MAPPING=2
uint8 STATE_MAP_READY=3
uint8 STATE_LOCALIZING=4
uint8 STATE_READY=5
uint8 STATE_NAVIGATING=6
uint8 STATE_STOPPED=7
uint8 STATE_FAULT=8

uint8 state
string map_id
string task_id
uint8 localization_state
bool safety_latched
string reason
