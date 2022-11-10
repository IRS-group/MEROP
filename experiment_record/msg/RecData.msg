# This message is to start and stop recording a rosbag of the system.
# It takes a bool that indicates the start/stop, and the mission type, the condition, and subject number.
bool rec # Switch to start or stop recording
string mission # Mission handle (INS, EXPL)
string cond # Condition handle (CTRL, HAF)
int32 subnum # Suject number of 00 type
