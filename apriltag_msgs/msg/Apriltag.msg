# Apriltag
# This message contains an Apriltag Detection
int32 id                       # Id of the apriltag in its tag family, eg. 0
string family                  # Tag family, eg. tf36h11
int32 hamming                  # Hamming distance
int32 border                   # Black border
int32 bits                     # Bits, tf36h11 has 6 bits
geometry_msgs/Point center     # Center of this apriltag
geometry_msgs/Point[4] corners # Corners of this apriltag
