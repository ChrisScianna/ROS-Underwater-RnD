Header header
float32 roll
float32 pitch
float32 yaw
float32 speed_knots
uint8 ena_mask

# Offsets into ena_mask for data fields
uint8 ROLL_ENA        = 1
uint8 PITCH_ENA       = 2
uint8 YAW_ENA         = 4
uint8 SPEED_KNOTS_ENA = 8
