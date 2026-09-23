# MotorState — single motor feedback state
# Published as part of MotorStateArray at state_publish_hz (default 200Hz).

std_msgs/Header header
uint8  channel          # CAN channel index
uint8  motor_index      # Motor slot index on the channel
uint16 can_id           # Motor CAN ID
float32 position        # Current position (rad), after sign + offset compensation
float32 velocity        # Current velocity (rad/s)
float32 torque          # Current torque (Nm)
uint8  error_flags      # Error flags from motor feedback
bool   enabled          # Whether motor is enabled (receiving commands)
bool   online           # Whether motor is online (receiving feedback)
