# A geographic point with uncertainty i.e. expressed as the first and
# second central moments of a probability vector distribution

geographic_msgs/GeoPoint position  # A 3x1 vector

# Row-major representation of the 3x3 covariance matrix
float64[9] covariance
