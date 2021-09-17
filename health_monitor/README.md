# Health Monitor package.
The health monitor tracks the diagnostics provided by nodes and generates a system fault with an error code if applicable. It also tracks if a node has crashed informed by [rosmon](http://wiki.ros.org/rosmon).  
After a fault has been set, it has to be cleared manually (usually through the OCU).  
Diagnostics are performed by each node using the `diagnostics_tool` package.  
The error/warning codes are defined in the [ReportFault message definition](msg/ReportFault.msg)
