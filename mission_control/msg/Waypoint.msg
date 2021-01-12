Header header
float32 latitude
float32 longitude
float32 depth
float32 altitude
float32 speed_knots
uint8 ena_mask

# Offsets into ena_mask for data fields
uint8 LAT_ENA   = 1
uint8 LONG_ENA  = 2
uint8 DEPTH_ENA = 4
uint8 ALTITUDE_ENA = 8
uint8 SPEED_KNOTS_ENA = 16
