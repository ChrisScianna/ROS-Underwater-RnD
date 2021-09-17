# Pose estimator
The pose estimator node consumes either INS or vectornav data to provide states. This option can be changed by modifying the ["use_ins" parameter](catkin_ws/src/private/system/ystem_bringup/launch/includes/pose_estimator.launch) in the pose_estimator launch file.

The node also performs these diagnostics:
  - The pitch angle threshold has been reached. (Error)
  - The depth threshold has been reached. (Error)
  - The pose estimator corrected data readings have stagnated. (Error)
  - The heading angle threshold has been reached. (Error)
  - The roll angle threshold has been reached. (Error)
  - The pose estimator data have stalled. (Warning)
