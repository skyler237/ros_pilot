# Offboard control command message

# control mode flags
uint8 MODE_DIRECT_CONTROL = 0
uint8 MODE_STABLE_CONTROL = 1

# ignore field bitmasks
uint8 IGNORE_NONE = 0
uint8 IGNORE_X = 1
uint8 IGNORE_Y = 2
uint8 IGNORE_Z = 4
uint8 IGNORE_F = 8

Header header
uint8 mode # offboard control mode for interpreting value fields
uint8 ignore # bitmask for ignore specific setpoint values
float32 x
float32 y
float32 z
float32 F
