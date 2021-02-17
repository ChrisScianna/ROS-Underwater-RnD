# A representation of the vehicle's short-term position and attitude variations with
# respect to some reference coordinate frame co-located with the vehicle's body
# coordinate frame (typicall) following NED conventions but not subject to rapid
# orientation changes.
#
# Seakeeping state is mostly concerned with high-frequency, external stimuli.
#
# See "Perez, T., & Fossen, T. I. (n.d.). Kinematics of Ship Motion. Ship Motion Control, 45–58."

string reference_frame_id  # Body co-located reference coordinate frame

auv_interfaces/MotionPerturbation perturbation
