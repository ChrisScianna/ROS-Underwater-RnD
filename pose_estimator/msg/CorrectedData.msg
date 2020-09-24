Header header
geographic_msgs/GeoPoint position   #latitude, longitude, altitude
geometry_msgs/Vector3 rpy_ang       # x y z -> r p y
float64 depth

# Indicies for rpy vectors
uint8 ROLL = 0
uint8 PITCH = 1
uint8 YAW = 2

float64[3] ssh_vel
float64[3] rpy_rate
float64 altitude
float64 speed

# Indicies for ssh vector
uint8 SURGE = 0
uint8 SWAY = 1
uint8 HEAVE = 2
