## `Fin_control` node
The `fin_control` node:
- Sets the angle sent by Jaus ROS Bridge or the autopilot (throug the [command actuator multiplexer](catkin_ws/src/public/cmd_actuators_mux/README.md)).
- Reports fins angles.

### Diagnostic
The `fin_control` node performs these diagnostics:
  - The reported angles data has stalled. (Warning)
  - The Fin angle data threshold has been reached. (Warning)
