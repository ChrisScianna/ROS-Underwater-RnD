Header header
float32 altitude
float32 heading
float32 speed_knots
uint8 ena_mask

# Offsets into ena_mask for data fields
uint8 ALTITUDE_ENA       = 1
uint8 HEADING_ENA     = 2
uint8 SPEED_KNOTS_ENA = 4
