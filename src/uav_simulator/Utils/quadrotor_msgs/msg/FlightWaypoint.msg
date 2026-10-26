Header header

# Per-waypoint planning policy. Values intentionally match TrajectoryPlan.msg.
uint8 MODE_UNSET=0
uint8 MODE_COLLISION_FREE=1
uint8 MODE_COLLISION=2

uint32 id
uint32 total_waypoints
geometry_msgs/Point position
uint8 flight_mode
float64 yaw
bool grasp_target
bool grasp_departure
