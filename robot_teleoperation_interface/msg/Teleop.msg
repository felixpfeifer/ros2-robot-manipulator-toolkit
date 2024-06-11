# This message is for the Communication between the Teleop Backend and the Teleop Frontend Interfaces

float64[] data_values [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 6 values for the 6 joints or the 3 Axis and 3 angles of a Point
int32 frame 0 # 0 for a Joint Pose a, 1 for a Cartesian Pose in world, 2 for a Cartesian Pose the Tool Frame
float64 speed # in m/s for the Cartesian Pose or in rad/s for the Joint Pose
bool[] gpios # Vector for the Gpios false if off and true if on
string target # Names Postions like home, init