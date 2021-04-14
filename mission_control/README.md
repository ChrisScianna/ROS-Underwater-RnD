# Mission Control
The mission control package provides an implementation of missions using behavior trees
It is compliant with the the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) library.  

## Features

- Topic interface for mission loading, execution, abortion, and monitoring, compliant with that of its predecessor (`mission_manager`)

- Actions supported are:
&nbsp; - **Abort**. Set the fins to surface and Thruster Velocity to 0 RPM.  
&nbsp; - **Attitude Servo**. It sends the command to actuator setting the roll, pitch, yaw and speed parameters.  
&nbsp; - **Fix Rudder**. It sends the command to actuators setting the rudder, depth and speed.  
&nbsp; - **Go To Waypoint**. It sends command to autopilot to go to altitude, latitude, longitude and speed.  
&nbsp; - **Payload Command**. It sends a string command to Payload  
&nbsp; - **Set Altitude Heading**. It sends command to actuator setting the altitude, heading and speed.  
&nbsp; - **Set Depth Heading**. It sends command to actuator setting the depth, heading and speed.  

- The missions can be designed using Groot
- Missions are described as behavior trees in XML
- Abort missions and go to surface in the event of unexpected anomalies (e.g. system faults)

## Instructions

- [How to use Groot?](doc/Groot/README.md)
