## `autopilot` node
The `autopilot` consists of control loops that actuate on setpoints that have been set by the mission control.

### Subscribed Topics

*/mngr/attitude_servo* ([mission_control/AttitudeServo](../mission_control/msg/AttitudeServo.msg)): Attitude servo setpoint.

*/mngr/altitude_heading* ([mission_control/AltitudeHeading](../mission_control/msg/AltitudeHeading.msg)): Altitude heading setpoint.

*/mngr/depth_heading* ([mission_control/DepthHeading](../mission_control/msg/DepthHeading.msg)): Depth heading setpoint.

*/mngr/fixed_rudder* ([mission_control/FixedRudder](../mission_control/msg/FixedRudder.msg)): Fix rudder setpoint.

*/mngr/waypoint* ([mission_control/Waypoint](../mission_control/msg/Waypoint.msg)): Waypoint setpoint.

*/mngr/report_heartbeat* ([/mission_contro/ReportHeartbeat](../mission_control/msg/ReportHeartbeat.msg)): The `autopilot` node receives on this topic regular pings from the `mission_control`. If the `autopilot` node doesn't receive a message from the `mission_control` node, it sets the thruster speed to 0 RPM and the fins to surface.

*/mngr/report_mission_execute_state* ([/mission_control/report_mission_execute_state](../mission_control/msg/ReportExecuteMissionState.msg)): The `autopilot` node receives on this topic the status of the mission. If the mission is complete, the `autopilot` node sets the thruster speed to 0 RPM and the fins to surface.

*/state* ([auv_interfaces/StateStamped](../auv_interfaces/msg/State.msg)): A full description of the vehicle's kinematic and dynamic state. It includes maneuvering and sea-keeping ship motion information as well as the vehicle's global position.

### Published Topics

*/input/autopilot/set_angles* ([fin_control/SetAngles](../fin_control/msg/SetAngles.msg)): Fins' angle positions command, subscribed by the [cmd_actuator_mux](../cmd_actuators_mux/README.md) node.

*/input/autopilot/set_rpm* ([thruster_control/SetRPM](../thruster_control/msg/SetRPM.msg)): Thruster's velocity command, subscribed by the [cmd_actuator_mux](../cmd_actuators_mux/README.md) node.
