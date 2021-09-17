# Mission Control
The mission control package provides an implementation of missions using behavior trees
Mission control uses the [BehaviorTree.CPP](https://www.behaviortree.dev/) library for mission definition. A Behavior Tree (BT) __is a way to structure the switching between different tasks__. This is nothing more than a mechanism to invoke callbacks at the right time under the right conditions.

## Features
- Topic interface for:
    - Abort Mission.
    - Execute Mission.
    - Load Mission.
    - Query Missions.
    - Remove Missions.
    - Report Execute Mission State.
    - Report Load Mission State.
    - Report Missions.

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

- The missions can be designed using Groot
- Missions are described as behavior trees in XML
- Abort missions and go to surface in the event of unexpected anomalies (e.g. system faults)

## Instructions

- [How to use Groot?](doc/Groot/README.md)
