# MotorCommand — single motor control command
# Used with ~/command topic for individual motor control.
# channel=255 means all channels, motor_index=255 means all motors on that channel.

uint8 MODE_DISABLE = 0
uint8 MODE_ENABLE  = 1
uint8 MODE_ZERO    = 2
uint8 MODE_RUN     = 3
# Tigertoe (钛虎) encoder motor modes:
uint8 MODE_BRAKE        = 4   # Send brake command (0x02)
uint8 MODE_GET_POSITION = 5   # Query encoder position (0x08)
uint8 MODE_SET_POSITION = 6   # Set target position (0x1E, uses position field)

uint8  channel        # CAN channel index: 0=left arm, 1=right arm, 2=tigertoe
uint8  motor_index    # Motor slot index on the channel (0-based)
uint8  mode           # Operating mode

float32 position       # Desired position (rad, or encoder cnt for SET_POSITION)
float32 velocity       # Desired velocity (rad/s, unused for tigertoe)
float32 kp             # Position gain (unused for tigertoe)
float32 kd             # Velocity gain (unused for tigertoe)
float32 torque         # Feed-forward torque (unused for tigertoe)
