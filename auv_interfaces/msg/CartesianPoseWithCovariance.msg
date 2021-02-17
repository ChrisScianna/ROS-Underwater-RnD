# A pose with uncertainty i.e. expressed as the first and second central
# moments of a probability vector distribution.

auv_interfaces/CartesianPose mean  # A 6x1 vector

# Row-major representation of the 6x6 covariance matrix
float64[36] covariance
