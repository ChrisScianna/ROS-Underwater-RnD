# This message holds the error value of the system
#

#Error Codes
uint64 PAYLOAD_ERROR = 1                                    #0x1
uint64 PAYLOAD_NODE_DIED = 2                                #0x2

uint64 PRESSURE_DATA_STALE = 4                              #0x4
uint64 PRESSURE_DATA_STAGNATED = 8                          #0x8
uint64 PRESSURE_THRESHOLD_REACHED = 16                      #0x10
uint64 PRESSURE_NODE_DIED = 32                              #0x20

uint64 AHRS_DATA_STALE = 64                                 #0x40
uint64 AHRS_DATA_STAGNATED = 128                            #0x80
uint64 AHRS_NODE_DIED = 256                                 #0x100

uint64 FIN_DATA_STALE = 512                                 #0x200
uint64 FIN_DATA_THRESHOLD_REACHED = 1024                    #0x400
uint64 FIN_NODE_DIED = 2048                                 #0x800

uint64 THRUSTER_RPM_STALE = 4096                            #0x1000
uint64 THRUSTER_RPM_THRESHOLD_REACHED = 8192                #0x2000
uint64 THRUSTER_TEMP_STALE = 16384                          #0x4000
uint64 THRUSTER_TEMP_STAGNATED = 32768                      #0x8000
uint64 THRUSTER_TEMP_THRESHOLD_REACHED = 65536              #0x10000
uint64 THRUSTER_NODE_DIED = 131072                          #0x20000

uint64 POSE_DATA_STALE = 262144                             #0x40000
uint64 POSE_DATA_STAGNATED = 524288                         #0x80000
uint64 POSE_ROLL_THRESHOLD_REACHED = 1048576                #0x100000
uint64 POSE_PITCH_THRESHOLD_REACHED = 2097152               #0x200000
uint64 POSE_HEADING_THRESHOLD_REACHED = 4194304             #0x400000
uint64 POSE_DEPTH_THRESHOLD_REACHED = 8388608               #0x800000
uint64 POSE_NODE_DIED = 16777216                            #0x1000000

uint64 AUTOPILOT_NODE_DIED = 33554432                       #0x2000000

uint64 JAUS_NODE_DIED = 67108864                            #0x4000000


uint64 BATTERY_INFO_STALE = 134217728                       #0x8000000
uint64 BATTERY_INFO_STAGNATED = 268435456                   #0x10000000
uint64 BATTERY_CURRENT_THRESHOLD_REACHED = 536870912        #0x20000000
uint64 BATTERY_VOLTAGE_THRESHOLD_REACHED = 1073741824       #0x40000000
uint64 BATTERY_TEMPERATURE_THRESHOLD_REACHED = 2147483648   #0x80000000
uint64 BATTERY_NODE_DIED = 4294967296                       #0x100000000

uint64 BATTERY_LEAK_DETECTED = 8589934592                   #0x200000000

uint64 MISSION_PARAMETERS_OUT_OF_RANGE = 17179869184        #0x400000000
uint64 MISSION_NODE_DIED = 34359738368                      #0x800000000

std_msgs/Header header
uint64 fault_id
