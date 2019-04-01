# sensor_msgs/JointState with an additional field for Motor Controller state
std_msgs/Header header

string[] name
float64[] position
float64[] velocity
float64[] acceleration
uint8[] motor_controller_state