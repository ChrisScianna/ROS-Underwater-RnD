## `autopilot` node
The `autopilot` consists of control loops that actuate on setpoints that have been set by the mission control.

These setpoints are:
- **Attitude Servo**. Roll, pitch, yaw and speed targets.
- **Altitude Heading**. Altitude, heading and speed targets.
- **Depth Heading**. Depth, heading and speed targets.
- **Fix Rudder**. Rudder, depth and speed targets.
- **Waypoint**. Altitude, latitude, longitude and speed targets.

The `autopilot` node checks that the `mission_control` node is running. If it detects that the `mission_control` node crashes, it sets the thruster velocity to 0 RPM and the fins to surface.
