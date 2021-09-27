## `Thruster control` node
The `thruster control` node manages the velocity of the thruster.  

### Subscribed Topics

*/thruster_control/set_rpm* ([thruster_control/SetRPM](msg/SetRPM.msg): Subscribes to the request of the velocity to the thruster.  


### Published Topics

*/thruster_control/report_battery_health* ([sensor_msgs/BatteryState](https://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html): Battery state.

*/thruster_control/report_motor_temperature* ([thruster_control/ReportMotorTemperature](msg/ReportMotorTemperature.msg): Motor temperature.

*/thruster_control/report_rpm* ([thruster_control/ReportRPM.msg](msg/ReportRPM.msg): Thruster velocity.

*/diagnostics* ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html): `thruster_control` node diagnostics
  - The thruster temperature threshold has been reached. (Error)
  - The thruster motor rpm report has stalled. (Warning)
  - The thruster temperature report has stalled. (Warning)
  - The thruster motor velocity threshold has been reached. (Warning)
  - The thruster temperature readings have stagnated. (Warning)
  - Battery current threshold has been reached. (Warning)
  - Battery voltage threshold has been reached. (Warning)
  - Battery report has stalled. (Warning)
  - Battery report readings have stagnated. (Warning)
