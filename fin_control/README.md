## `Fin_control` node
The `fin_control` node:
- Sets the angle sent by Jaus ROS Bridge or the autopilot (throug the [command actuator multiplexer](catkin_ws/src/public/cmd_actuators_mux/README.md)).
- Reports fins angles.

### Subscribed Topics

*/fin_control/set_angles* ([fin_control/SetAngles](msg/SetAngles.msg)): Subscription to fins angles request topic.

### Published Topics

*/fin_control/report_angle* ([fin_control/ReportAngle](msg/ReportAngle.msg)): Reports fins angles.

*/diagnostics* ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html)): `fin_control` node diagnostics

  - The reported angles data has stalled. (Warning)
  - The Fin angle data threshold has been reached. (Warning)
