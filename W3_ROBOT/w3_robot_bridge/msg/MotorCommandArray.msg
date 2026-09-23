# MotorCommandArray — batch motor control command
# Used with ~/commands topic for efficient group control.
# Sending multiple commands in one message avoids frequent ROS API calls
# and reduces overhead when controlling many motors at once.

std_msgs/Header header
MotorCommand[] commands
