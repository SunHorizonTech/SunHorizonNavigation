std_msgs/Header header

uint8 STATE_UNKNOWN=0
uint8 STATE_INITIALIZING=1
uint8 STATE_HEALTHY=2
uint8 STATE_DEGRADED=3
uint8 STATE_LOST=4

uint8 state
string backend
bool measurement_accepted
float64 score
uint32 iterations
float64 position_covariance
float64 yaw_covariance
float64 measurement_age_sec
string reason
