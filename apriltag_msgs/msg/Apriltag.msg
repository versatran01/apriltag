# Apriltag
# This message contains a detected Apriltag
int32 id                       # Id of the apriltag in its tag family
string family                  # Header should be the same is the detection image
float64 size                   # Size of the Apriltag in meter
int32 hamming                  # Hamming distance
int32 border                   # Black border
int32 bits                     # Bits, 36h11 has 6 bits
bool estimated                 # Is pose estimated
geometry_msgs/Point center
geometry_msgs/Point[] corners
geometry_msgs/Pose pose        # Pose of this Apriltag in camera frame
