# This message contains a detected Apriltag
uint32 id                      # id of the apriltag in its tag family
string family                  # Header should be the same is the detection image
float32 size                   # Size of the Apriltag in meter
uint32 hamming_distance        # hamming distance
geometry_msgs/Point center
geometry_msgs/Point[] corners
geometry_msgs/Pose pose        # Pose of this Apriltag in camera frame

