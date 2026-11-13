std_msgs/Header header

uint8 STATE_UNKNOWN=0
uint8 STATE_INITIALIZING=1
uint8 STATE_TRACKING=2
uint8 STATE_DEGRADED=3
uint8 STATE_LOST=4

uint8 state
string backend
bool tracking
bool timestamp_monotonic
bool pose_continuous
float64 input_age_sec
float64 tracking_lost_duration_sec
float64 last_translation_jump_m
float64 last_rotation_jump_rad
string reason
