# A representation of the vehicle's position and attitude with respect to some
# reference coordinate frame fixed to the Earth (typically) at mean water-free
# surface and (also typically) following SWD conventions i.e. its x-axis
# pointing towards South, its y-axis pointing towards West, an its z-axis
# pointing towards the centre of the Earth.
#
# Manoeuvring state is mostly concerned with low-frequency, actuated motion.
#
# See "Perez, T., & Fossen, T. I. (n.d.). Kinematics of Ship Motion. Ship Motion Control, 45–58."

string reference_frame_id  # Fixed terrestrial reference coordinate frame

auv_interfaces/CartesianPoseWithCovariance pose
auv_interfaces/GeneralizedVectorWithCovariance velocity
