## `Pressure sensor` node
The `pressure sensor` node is a driver ROS node for Keller 9Lx pressure sensor.

### Published Topics

*/pressure_sensor/pressure* ([sensor_msgs/FluidPressure](http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html)): Pressure sensor data.

*/pressure_sensor/temperature* ([sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)): Temperature data.

*/diagnostics* ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html)): `pressure sensor` node diagnostics
  - The pressure sensor readings have stagnated. (Error)
  - The pressure sensor data has stalled. (Error)
  - The pressure sensor threshold has been reached. (Error)
