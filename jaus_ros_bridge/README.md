## `Jaus ROS bridge` node
The `Jaus ROS bridge` node manages the communication between the OCU and:
- Actuators:
  -  Fin Control.
  -  Thruster Control.
- Mission Control.
- Sensors


### Subscribed Topics

*/health_monitor/report_fault*([health_monitor/ReportFault](../health_monitor/msg/ReportFault.msg))  
Receives the error code reported by the health monitor.

*/fin_control/report_angle* ([fin_control/ReportAngle](../fin_control/msg/ReportAngle.msg))  
Subscribes to the fins angles report.

*/mngr/report_mission_execute_state* ([mission_control/ReportExecuteMissionState](../mission_control/msg/ReportExecuteMissionState.msg))  
Subscribes to check the status of the mission.

*/mngr/report_mission_load_state* ([mission_control/ReportLoadMissionState](../mission_control/msg/ReportLoadMissionState.msg))  
Subscribes to check if the mission has been loaded correctly.

*/mngr/report_missions* ([mission_control/ReportMissions](../mission_control/msg/ReportMissions.msg))  
Receives the missions that have been loaded.

*/pressure_sensor/pressure* ([sensor_msgs/FluidPressure](http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html))
Subscribes to the pressure sensor report.

*/state* ([auv_interfaces/StateStamped](../auv_interfaces/msg/State.msg))  
A full description of the vehicle's kinematic and dynamic state. It includes manoeuvring and seakeeping ship motion information as well as the vehicle's global position.

*/thruster_control/report_battery_health* ([sensor_msgs/BatteryState](https://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html))  
Subscribes to the battery state.

*/thruster_control/report_motor_temperature* ([thruster_control/ReportMotorTemperature](../thruster_control/msg/ReportMotorTemperature.msg))  
Subscribes to the motor temperature.

*/thruster_control/report_rpm* ([thruster_control/ReportRPM.msg](../thruster_control/msg/ReportRPM.msg))  
Subscribes to the thruster velocity.

### Published Topics

*/health_monitor/clear_fault* ([health_monitor/ClearFault](../health_monitor/msg/ClearFault.msg))  
Clears the faults.

*/input/jaus_ros_bridge/set_angles* ([fin_control/SetAngles](../fin_control/msg/SetAngles.msg))  
Publishes the fins angle position.

*/input/jaus_ros_bridge/set_rpm* ([thruster_control/SetRPM](../thruster_control/msg/SetRPM.msg))  
Publishes the Thruster velocity.

*/jaus_ros_bridge/enable_logging* ([jaus_ros_bridge/EnableLogging](msg/EnableLogging.msg))  
Publishes the enable logging flag.

*/mngr/abort_mission* ([mission_control/AbortMission](../mission_control/msg/AbortMission.msg))  
Publishes an abort mission command.

*/mngr/execute_mission* ([mission_control/ExecuteMission](../mission_control/msg/ExecuteMission.msg))  
Publishes an execute mission command.

*/mngr/load_mission* ([mission_control/LoadMission](../mission_control/msg/LoadMission.msg))  
Publishes a load mission command.

*/mngr/remove_missions* ([mission_control/RemoveMissions](../mission_control/msg/RemoveMissions.msg))  
Publishes a remove mission command.

*/payload_manager/command* ([payload_manager/PayloadCommand](../payload_manager/msg/PayloadCommand.msg))  
Sends command to the payload.
