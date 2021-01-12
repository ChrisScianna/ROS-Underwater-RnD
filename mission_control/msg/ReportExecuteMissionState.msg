uint8 ERROR = 0
uint8 ABORTING = 1
uint8 COMPLETE = 2
uint8 PAUSED = 3
uint8 EXECUTING = 4

std_msgs/Header header
uint8 mission_id
uint8 execute_mission_state
string current_behavior_name
uint16 current_behavior_id
