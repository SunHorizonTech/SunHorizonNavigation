# Versioned observation evidence, NOT motion authorization or stop-latch reset.
# coverage.header is the single timestamp/frame for this complete calculation.
# Original coverage booleans retain their semantics even for GeometryHold.
sunhorizon_navigation_interfaces/ObstacleCoverageStatus coverage
uint8 schema_version
# 0=HardFault, 1=Clear, 2=GeometryHold; any unknown value must fail closed.
uint8 disposition
bool non_region_healthy
bool region_evaluation_complete
# 0=Rejected, 1=Clear, 2=Obstacle, 3=IncompleteRays.
uint8 region_evaluation_code
# Same 0/1/2 encoding as disposition, after region source/receipt freshness checks.
uint8 region_disposition
int64 region_source_stamp_ns
uint64 region_frame_sequence
uint64 region_evaluation_sequence
string region_contract
