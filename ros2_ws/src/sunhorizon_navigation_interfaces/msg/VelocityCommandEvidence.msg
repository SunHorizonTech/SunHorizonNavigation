# A single command envelope per hop. Origin fields MUST be copied unchanged by
# collision_monitor; its process/receipt time MUST NOT replace generation time.
uint8 SCHEMA_V1=1
uint8 GENERATED_CONTROLLER=1
uint8 GENERATED_EXPLICIT_ZERO=2
uint8 GENERATED_PLUGIN_FAILURE_ZERO=3
uint8 COLLISION_NOT_PROCESSED=0
uint8 COLLISION_CLEAR=1
uint8 COLLISION_APPROACH_SLOWDOWN=2
uint8 COLLISION_STOP=3
uint8 COLLISION_SOURCE_INVALID=4
uint8 COLLISION_ORIGIN_INVALID=5
uint8 schema_version
uint8[16] controller_epoch_uuid
uint64 command_sequence
# Diagnostic generation segment only; normal same-NavigateTo path updates can
# change this value. NavigationStatus.task_id/server ownership bind the task.
uint64 controller_goal_sequence
# Linux boot_id identifies the CLOCK_MONOTONIC domain shared on one robot host.
string clock_boot_id
string frame_id
# Generation interval starts BEFORE obtaining pose/progress/odometry inputs.
# Plugin TwistStamped.header is observation time, never generation time.
int64 compute_begin_ros_ns
int64 compute_end_ros_ns
int64 compute_begin_steady_ns
int64 compute_end_steady_ns
uint8 generation_state
geometry_msgs/Twist origin_velocity
# Controller: exactly origin_velocity. Collision: result of the same callback.
geometry_msgs/Twist velocity
uint8 collision_state
uint8[16] collision_epoch_uuid
uint64 collision_sequence
int64 collision_begin_ros_ns
int64 collision_end_ros_ns
int64 collision_begin_steady_ns
int64 collision_end_steady_ns
