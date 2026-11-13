# One immutable sensor observation used by a spatial evaluation. Same host/boot
# monotonic clock as the producer. A new publication never refreshes these times.
uint8 BELLY=0
uint8 CHEST=1
uint8 AIRY=2
uint8 CONTRIBUTES_FREE=1
uint8 CONTRIBUTES_OCCUPIED=2
uint8 source_id
string producer_epoch
string frame_id
uint64 frame_sequence
int64 source_ros_ns
int64 first_receipt_ros_ns
int64 first_receipt_steady_ns
int64 valid_until_ros_ns
int64 valid_until_steady_ns
# Bitmask; zero means no spatial contribution, not observed free space.
uint8 contribution
bool geometry_valid
