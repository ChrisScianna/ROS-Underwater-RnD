# Command Actuator Multiplexer.
A multiplexer for command thruster velocity and fins inputs. It arbitrates incoming messages from the Jaus ROS bridge and autopilot, allowing one node at a time to command the actuator, based on priorities.
It also deactivate the current allowed topic if no messages are received after a configured timeout. All topics, together with their priority and timeout are configured through a YAML file.

![cmd mux diagram](doc/cmd_multiplexer_diagram.png "Diagram")

The multiplexer node publishes which input is in control on the topic "active":
```
/CmdActuatorMuxNodelet/fin_angle_active
/CmdActuatorMuxNodelet/set_rpm_active
```

## Configuration
The configuration of the _Command Multiplexer_ is provided as a [YAML file](param/actuator_mux.yaml).
