## `Health monitor` node.
The `health monitor` node tracks the diagnostics provided by nodes and generates a system fault with an error code if applicable. It also tracks if a node has crashed informed by [rosmon](http://wiki.ros.org/rosmon).  
After a fault has been set, it has to be cleared manually (usually through the OCU).  
Diagnostics are performed by each node using the `diagnostics_tool` package.  
The error/warning codes are defined in the [ReportFault message definition](msg/ReportFault.msg)

### Subscribed Topics

*/health_monitor/clear_fault* ([health_monitor/ClearFault](msg/ClearFault.msg): Clears the faults.

*/diagnostics* ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html): subscribe to the diagnostic information sent by nodes

*/rosmon/state* ([rosmon/State](https://github.com/xqms/rosmon/blob/master/rosmon_msgs/msg/State.msg): Current state of all controlled nodes.

### Published Topics

*/health_monitor/report_fault* ([health_monitor/ReportFault](msg/ReportFault.msg): Error codes, subscribed by the [jaus_ros_bridge](../jaus_ros_bridge/README.md) node.