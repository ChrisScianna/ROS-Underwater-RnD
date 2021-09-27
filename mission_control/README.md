## `Mission control` node
The `mission control` node provides an implementation of missions using behavior trees
`Mission control` uses the [BehaviorTree.CPP](https://www.behaviortree.dev/) library for mission definition. A Behavior Tree (BT) __is a way to structure the switching between different tasks__. This is nothing more than a mechanism to invoke callbacks at the right time under the right conditions.

## Features
- Missions can be designed using [Groot](https://github.com/BehaviorTree/Groot)
- Missions are described as behavior trees in XML
- Safe mission aborts -- vehicle is commanded to surface in the event of unexpected anomalies (e.g. system faults)

- Actions supported are:
    - [Attitude servo.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/attitude_servo.h)
    - [Altitude heading.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/set_altitude_heading.h)
    - [Depth heading.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/set_depth_heading.h)
    - [Fixed rudder.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/fix_rudder.h)
    - [Go to waypoint.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/go_to_waypoint.h)
    - [Payload command.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/payload_command.h)
    - [Abort.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/abort.h)
    - [Log to bagfile.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/log_to_bagfile.h) (Decorator)
    - [Delay.](../catkin_ws/src/public/mission_control/include/mission_control/behaviors/delay_for.h) (Decorator)

## Instructions

- [How to use Groot?](doc/Groot/README.md)


### Subscribed Topics

*/mngr/abort_mission* ([mission_control/AbortMission](msg/AbortMission.msg): Receives the abort mission command.

*/mngr/execute_mission* ([mission_control/ExecuteMission](msg/ExecuteMission.msg): Execute a mission.

*/mngr/load_mission* ([mission_control/LoadMission](msg/LoadMission.msg): Load a mission located  on a path.

*/mngr/remove_missions* ([mission_control/RemoveMissions](msg/RemoveMissions.msg): Receives the command to remove mission.

*/mngr/query_missions* ([mission_control/QueryMissions](msg/QueryMissions.msg): Receives the query mission command.

*/mngr/stop_missions* ([std_msgs/Empty](https://docs.ros.org/en/api/std_msgs/html/msg/Empty.html): Receives a command to stop the mission.

### Published Topics

*/mngr/attitude_servo* ([mission_control/AttitudeServo](msg/AttitudeServo.msg): Attitude servo setpoint.

*/mngr/altitude_heading* ([mission_control/AltitudeHeading](msg/AltitudeHeading.msg): Altitude heading setpoint.

*/mngr/depth_heading* ([mission_control/DepthHeading](msg/DepthHeading.msg): Depth heading setpoint.

*/mngr/fixed_rudder* ([mission_control/FixedRudder](msg/FixedRudder.msg): Fix rudder setpoint.

*/mngr/waypoint* ([mission_control/Waypoint](msg/Waypoint.msg): Waypoint setpoint.

*/mngr/report_heartbeat* ([/mission_contro/ReportHeartbeat](msg/ReportHeartbeat.msg): Ping to be received by the `autopilot` node. If the `autopilot` node doesn't receive the message form the `mission_control` node, the `autopilot` node sets the thruster velocity to 0 RPM and the fins to surface.

*/mngr/report_mission_execute_state* ([mission_control/ReportExecuteMissionState](msg/ReportExecuteMissionState.msg): Status of the mission.

*/mngr/report_mission_load_state* ([mission_control/ReportLoadMissionState](msg/ReportLoadMissionState.msg): Status of the load mission command.

*/mngr/report_missions* ([mission_control/ReportMissions](msg/ReportMissions.msg): Set of missions that have been loaded.
