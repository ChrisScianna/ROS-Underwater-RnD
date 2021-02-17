# A representation of the vehicle's position and attitude with respect to some
# reference coordinate frame fixed to the Earth (typically) at mean water-free
# surface and (also typically) following NED conventions i.e. its x-axis
# pointing towards North, its y-axis pointing towards East, an its z-axis
# pointing towards the centre of the Earth.
#
# Manoeuvring state is mostly concerned with low-frequency, actuated motion.
#
# See "Perez, T., & Fossen, T. I. (n.d.). Kinematics of Ship Motion. Ship Motion Control, 45–58."

string reference_frame_id  # Fixed terrestrial reference coordinate frame

auv_interfaces/CartesianPoseWithCovariance pose
auv_interfaces/GeneralizedVectorWithCovariance velocity
