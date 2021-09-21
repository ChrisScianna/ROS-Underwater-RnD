## `Pose estimator` node
The `pose estimator` node consumes either INS or vectornav data to provide states. This option can be changed by modifying the ["use_ins" parameter](catkin_ws/src/private/system/ystem_bringup/launch/includes/pose_estimator.launch) in the `pose estimator` launch file.

### Published Topics

*/diagnostics* ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html))  
Publishes the `Pose estimator` node diagnostics:

  - The pose estimator data have stalled. (Warning)
  - The pose estimator corrected data readings have stagnated. (Error)
  - The roll angle threshold has been reached. (Error)
  - The pitch angle threshold has been reached. (Error)
  - The heading angle threshold has been reached. (Error)
  - The depth threshold has been reached. (Error)

*/state* ([auv_interfaces/StateStamped](../auv_interfaces/msg/State.msg))  
Publishes a full description of the vehicle's kinematic and dynamic state. It includes manoeuvring and seakeeping ship motion information as well as the vehicle's global position.

### Subscribed Topics
*/pressure_sensor/pressure* ([sensor_msgs/FluidPressure](http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html))
Subscribes to the pressure sensor data.

*/thruster_control/report_rpm* ([thruster_control/ReportRPM.msg](../thruster_control/msg/ReportRPM.msg))  
Subscribes to the thruster velocity report.

*/vectornav/IMU* ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))  
Subscribes to the IMU data.

*/fix* ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html))
Navigation Satellite fix for GPS
