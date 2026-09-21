Header header

# Representation of the trajectory payload.
uint8 REP_WAYPOINTS=0        # waypoints are interpolation points; segment_times are per-segment durations
uint8 REP_BSPLINE=1          # waypoints are B-spline control points; segment_times are knot-span durations

uint8 representation
uint8 bspline_degree

uint8 MODE_UNSET=0
uint8 MODE_COLLISION_FREE=1
uint8 MODE_COLLISION=2

uint8 trajectory_mode

geometry_msgs/Point[] waypoints
geometry_msgs/Vector3[] headings
bool[] position_constraints

float64[] segment_times
