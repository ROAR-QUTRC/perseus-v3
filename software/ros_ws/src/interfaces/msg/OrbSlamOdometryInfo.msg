# Diagnostics for one vision/orb_slam_odometry pass, published alongside its odometry so a
# consumer can tell a confident step from a degraded or discarded one.

std_msgs/Header header

# ORB-SLAM3's Tracking::eTrackingState after this pass.
int8 STATE_SYSTEM_NOT_READY = -1
int8 STATE_NO_IMAGES_YET = 0
int8 STATE_NOT_INITIALIZED = 1
int8 STATE_OK = 2
int8 STATE_RECENTLY_LOST = 3
int8 STATE_LOST = 4
int8 STATE_OK_KLT = 5
int8 tracking_state

# Map points matched in this frame: how well constrained the pose is. The published
# covariance grows as this falls below reference_map_points.
int32 tracked_map_points

# True when this pass's camera step went into the odometry with tracking covariance.
# False means an identity step with the lost-tracking covariance was published instead:
# not tracking, the first frame of a new track, a map change, or a rejected step.
bool step_published

# Tracking said OK, but the step exceeded max_step_translation_m / max_step_rotation_rad
# -- a relocalisation or map switch that ORB-SLAM3 did not flag -- so it was discarded.
bool step_rejected

# ORB-SLAM3 reported a map change (a merge or a global bundle adjustment) this pass; the
# step across it is discarded since the map's frame moved under it.
bool map_changed

# Position variance (m^2) published with this pass, per axis.
float64 position_variance

# Wall-clock time spent in ORB-SLAM3 for this pass, in seconds.
float64 runtime_s
