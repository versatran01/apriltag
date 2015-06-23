# Apriltag
# This message contains a detected Apriltag
int32 id                       # Id of the apriltag in its tag family
string family                  # Header should be the same is the detection image
int32 hamming                  # Hamming distance
int32 border                   # Black border
int32 bits                     # Bits, 36h11 has 6 bits
geometry_msgs/Point center     # Center of this apriltag
geometry_msgs/Point[4] corners # Corners of this apriltag
                               #
