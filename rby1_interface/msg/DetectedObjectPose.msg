# One valid 6-DoF object observation.
# header.stamp is the image capture/detection time and header.frame_id is the
# source coordinate frame in which pose is expressed.
std_msgs/Header header

# Stable identifier selected by the perception producer (for example, tag_0).
string object_id

# Object pose relative to header.frame_id. Position is in metres and
# orientation is a normalized quaternion in x, y, z, w field order.
geometry_msgs/Pose pose

# Normalized detector confidence in the inclusive range [0.0, 1.0].
float32 confidence
