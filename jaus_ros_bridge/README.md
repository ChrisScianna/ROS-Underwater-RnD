## `Jaus ROS bridge` node
The `Jaus ROS bridge` node manages the communication between the OCU and:
- Actuators:
  -  Fin Control.
  -  Thruster Control.
- Mission Control.
- Sensors


### Subscribed Topics

*/health_monitor/report_fault*([health_monitor/ReportFault](../health_monitor/msg/ReportFault.msg): Receives the error code reported by the health monitor.

*/fin_control/report_angle* ([fin_control/ReportAngle](../fin_control/msg/ReportAngle.msg): Subscribes to the fins angles report.

*/mngr/report_mission_execute_state* ([mission_control/ReportExecuteMissionState](../mission_control/msg/ReportExecuteMissionState.msg): Subscribes to check the status of the mission.

*/mngr/report_mission_load_state* ([mission_control/ReportLoadMissionState](../mission_control/msg/ReportLoadMissionState.msg): Subscribes to check if the mission has been loaded correctly.

*/mngr/report_missions* ([mission_control/ReportMissions](../mission_control/msg/ReportMissions.msg): Receives the missions that have been loaded.

*/pressure_sensor/pressure* ([sensor_msgs/FluidPressure](http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html)): Subscribes to the pressure sensor report.

*/state* ([auv_interfaces/StateStamped](../auv_interfaces/msg/State.msg): A full description of the vehicle's kinematic and dynamic state. It includes manoeuvring and seakeeping ship motion information as well as the vehicle's global position.

*/thruster_control/report_battery_health* ([sensor_msgs/BatteryState](https://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html): Subscribes to the battery state.

*/thruster_control/report_motor_temperature* ([thruster_control/ReportMotorTemperature](../thruster_control/msg/ReportMotorTemperature.msg): Subscribes to the motor temperature.

*/thruster_control/report_rpm* ([thruster_control/ReportRPM.msg](../thruster_control/msg/ReportRPM.msg): Subscribes to the thruster velocity report.

### Published Topics

*/health_monitor/clear_fault* ([health_monitor/ClearFault](../health_monitor/msg/ClearFault.msg): Clear the faults command, subscribed by the [health_monitor](../health_monitor/README.md) node.

*/input/jaus_ros_bridge/set_angles* ([fin_control/SetAngles](../fin_control/msg/SetAngles.msg): Fins' angle positions command, subscribed by the [cmd_actuator_mux](../cmd_actuators_mux/README.md) node.

*/input/jaus_ros_bridge/set_rpm* ([thruster_control/SetRPM](../thruster_control/msg/SetRPM.msg): Thruster's velocity command, subscribed by the [cmd_actuator_mux](../cmd_actuators_mux/README.md) node.


*/jaus_ros_bridge/enable_logging* ([jaus_ros_bridge/EnableLogging](msg/EnableLogging.msg): Enable logging flag.

*/mngr/abort_mission* ([mission_control/AbortMission](../mission_control/msg/AbortMission.msg): Abort mission command, subscribed by the [mission_control](../mission_control/README.md) node.

*/mngr/execute_mission* ([mission_control/ExecuteMission](../mission_control/msg/ExecuteMission.msg): Execute mission command, subscribed by the [mission_control](../mission_control/README.md) node.

*/mngr/load_mission* ([mission_control/LoadMission](../mission_control/msg/LoadMission.msg): Load mission command, subscribed by the [mission_control](../mission_control/README.md) node.

*/mngr/remove_missions* ([mission_control/RemoveMissions](../mission_control/msg/RemoveMissions.msg): Remove mission command, subscribed by the [mission_control](../mission_control/README.md) node.

*/payload_manager/command* ([payload_manager/PayloadCommand](../payload_manager/msg/PayloadCommand.msg): Payload command, subscribed by the [payload_manager](../payload_manager/README.md) node.
