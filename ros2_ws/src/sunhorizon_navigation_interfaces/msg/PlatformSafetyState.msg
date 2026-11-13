std_msgs/Header header
bool mode_allowed
string mode_name
bool ethercat_op
bool wkc_normal
bool joint_feedback_fresh
bool leg_joints_error_free
bool leg_joints_operation_enabled
# Deprecated wire-layout slot; always false in the navigation adapter.
# Do not use as a readiness, reset or motion gate. Hardware stop is platform-owned.
bool estop_clear
bool platform_fault
bool motion_ready
bool navigation_mode_discovered
int32 navigation_mode
float64 waist_yaw_rad
string reason
