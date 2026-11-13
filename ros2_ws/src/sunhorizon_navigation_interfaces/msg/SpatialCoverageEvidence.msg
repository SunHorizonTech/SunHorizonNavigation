# Opt-in spatial contract. Exactly one frozen evaluation per new belly image.
# It never changes the legacy region-depth diagnostic or schema-1 decision.
std_msgs/Header header
uint8 schema_version
string contract
string query_digest
string producer_epoch
string clock_boot_id
int64 anchor_source_ros_ns
uint64 anchor_frame_sequence
uint64 evaluation_sequence
uint64 input_generation
int64 evaluation_steady_ns
bool evaluation_complete
uint32 total_cells
uint32 free_cells
uint32 occupied_cells
uint32 unknown_cells
SpatialSourceEvidence[<=3] sources
string reason
