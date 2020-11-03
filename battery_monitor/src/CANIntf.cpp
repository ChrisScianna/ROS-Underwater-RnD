/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Original version: Christopher Scianna <Christopher.Scianna@us.QinetiQ.com>

#include <iostream>

#include <ros/ros.h>
#include "battery_monitor/CANIntf.h"
#include "battery_monitor/CO_VehicleSBC.h"

#define CAN_BUS_NAME "1"
#define CAN_BAUD "500K"
#define DEFAULT_HEARTBEAT_SLEEP_TIME_IN_MICROSECONDS 10000
#define DEFAULT_NODE_STATUS_SLEEP_TIME_IN_MICROSECONDS 200000
#define DEFAULT_VEHICLE_INFO_SLEEP_TIME_IN_MICROSECONDS 10000


void InitNode(CO_Data *d, UNS32 id);
void Callback_Initialisation(CO_Data *d);
void Callback_PreOperational(CO_Data *d);
void Callback_Operational(CO_Data *d);
void Callback_SlaveStateChange(CO_Data *d, UNS8 nodeId, e_nodeState newNodeState);
void Exit(CO_Data *d, UNS32 id);

CAN_PORT canport_; // CANFestival File descriptor

static std::string canBusName = CAN_BUS_NAME;
static std::string canBaud = CAN_BAUD;
static int updateHeartbeatSleepTimeInMicroseconds = DEFAULT_HEARTBEAT_SLEEP_TIME_IN_MICROSECONDS;
static int monitorNodeStatusSleepTimeInMicroseconds = DEFAULT_NODE_STATUS_SLEEP_TIME_IN_MICROSECONDS;
static int updateVehicleInfoSleepTimeInMicroseconds = DEFAULT_VEHICLE_INFO_SLEEP_TIME_IN_MICROSECONDS;


CANIntf::CANIntf() :
previousHeartbeat(0),
isInitialized(false),  
DMCHeartbeatTimeout(true),
lastPosition(0),
desiredPosition(0),
leakDetected(false),
sBCCommsLoss(false)
{
}

CANIntf::~CANIntf()
{
    if (isInitialized == true) 
    {
        // Stop the timer thread
        StopTimerLoop(&Exit);

        // Close the CAN port
        canClose(&CO_VehicleSBC_Data);

        TimerCleanup();

        isInitialized = false;

        // Stop & join class-created threads
        int retVal = 0;
        retVal = pthread_join(heartbeatUpdateThread, NULL);
        if (retVal)
          ROS_ERROR("CANopenInterface::shutdownThread [ERROR]: Cannot shutdown heartbeatUpdateThread thread. Error = %d\n", retVal);

        retVal = pthread_join(nodeStatusMonitorThread, NULL);
        if (retVal)
          ROS_ERROR("CANopenInterface::shutdownThread [ERROR]: Cannot shutdown nodeStatusMonitorThread thread. Error = %d\n", retVal);

        retVal = pthread_join(vehicleInfoUpdateThread, NULL);
        if (retVal)
          ROS_ERROR("CANopenInterface::shutdownThread [ERROR]: Cannot shutdown batteryPositionUpdateThread thread. Error = %d\n", retVal);
  }
}

bool CANIntf::Init()
{
    if (isInitialized == true)
    {
        return true;
    }

	lastPosition = 0;


    isInitialized = SetupCANBus();

    if (isInitialized == true)
    {
        // Start the threads
        int retVal = 0;

        retVal = pthread_create(&heartbeatUpdateThread, NULL, UpdateHeartbeat, (void *)this);
        if (retVal != 0) 
        {
            ROS_ERROR("CANopenInterface::Init [ERROR]: heartbeatUpdateThread - Cannot start thread\n");
            return false;
        }

        retVal = pthread_create(&nodeStatusMonitorThread, NULL, MonitorNodeStatus, (void *)this);
        if (retVal != 0) 
        {
            ROS_ERROR("CANopenInterface::Init [ERROR]: nodeStatusMonitorThread - Cannot start thread\n");
            return false;
        }

        retVal = pthread_create(&vehicleInfoUpdateThread, NULL, UpdateVehicleInfo, (void *)this);
        if (retVal != 0) 
        {
            ROS_ERROR("CANopenInterface::Init [ERROR]: vehicleInfoUpdateThread - Cannot start thread\n");
            return false;
        }
    }

    return isInitialized;
}

bool CANIntf::SetupCANBus()
{
    char * tempCanBusName = const_cast<char*> ( canBusName.c_str() );
    char * tempCanBaud = const_cast<char*> ( canBaud.c_str() );

    // Generate the board config
    s_BOARD config = {tempCanBusName, tempCanBaud};

    // Setup the callbacks
    CO_VehicleSBC_Data.initialisation = Callback_Initialisation;
    CO_VehicleSBC_Data.preOperational = Callback_PreOperational;
    CO_VehicleSBC_Data.operational = Callback_Operational;
    CO_VehicleSBC_Data.post_SlaveStateChange = Callback_SlaveStateChange;

    // Initialize the stack timer
    TimerInit();

    // Open the CAN port
    ROS_INFO("CANopenInterface::Init [INFO]: Opening CAN port");
    canport_ = canOpen(&config, &CO_VehicleSBC_Data);

    if (canport_ == NULL) 
    {
        ROS_ERROR("CANopenInterface::Init [ERROR]: Unable to open CAN port");
        return false;
    }

    // Start the timer thread
    StartTimerLoop(&InitNode);

    return true;
}

void *CANIntf::UpdateHeartbeat(void *data) 
{
  CANIntf *instance;
  instance = (CANIntf *)data;

  while (instance->IsInitialized()) 
  {
    EnterMutex();       // defined in CanFestival library
    MasterHeartbeat++;  // part of CO_VehicleSBC.h which is generated code from CanFestival 
    LeaveMutex();
    usleep(updateHeartbeatSleepTimeInMicroseconds); // time in microseconds
  }
}

void CANIntf::MonitorHeartbeat() 
{
    // PowerInterfaceBoardHeartbeat is defined in CO_VehicleSBC.h which is generated code. It is generated from python script called objdictedit.py
    if(PowerInterfaceBoardHeartbeat == previousHeartbeat)
    {
        DMCHeartbeatTimeout = true;
    }
    else
    {
        DMCHeartbeatTimeout = false;
    }

    previousHeartbeat = PowerInterfaceBoardHeartbeat;

}

void *CANIntf::MonitorNodeStatus(void *data) 
{
  CANIntf *instance;
  instance = (CANIntf *)data;

  while (instance->IsInitialized()) 
  {
    instance->MonitorHeartbeat();
    usleep(monitorNodeStatusSleepTimeInMicroseconds); // time in microseconds
  }
}

void CANIntf::GetVehicleStatusData()
{
    unsigned short swMajorVersion = 0;
    unsigned short swMinorVersion = 0;
    unsigned short swMicroVersion = 0;
    unsigned short swBuildVersion = 0;
    unsigned short motorStatus = 0;

	BatteryInfo temp;

	bool temp_leakDetected;
	bool temp_sBCCommsLoss;
	unsigned short temp_cur_position = 0;



    // getting info from CAN bus
    // general form --> localValue = CANBusValue
    EnterMutex(); // CAN Mutex

    // we don't do anything with these values
    swMajorVersion = SoftwareMajorVersion;
    swMinorVersion = SoftwareMinorVersion;
    swMicroVersion = SoftwareMicroVersion;
    swBuildVersion = SoftwareBuildVersion;

    // read values from CAN
    // Enum: {COMPLETE = 0, MOVING = 1, FAIL = 2}
    motorStatus = MotorCommandedStatus;
	temp_cur_position = MotorCurrentPosition;

	temp.batteryPackAConnected = BatteryPackAConnected;
	temp.batteryPackBConnected = BatteryPackBConnected;
	temp.batteryPacksTotalCurrent = static_cast<double>(BatteryPacksTotalCurrent) / (1LL << 10);

	temp.batteryPackACell1 = static_cast<double>(BatteryPackACell1) / (1LL << 10);
	temp.batteryPackACell2 = static_cast<double>(BatteryPackACell2) / (1LL << 10);
	temp.batteryPackACell3 = static_cast<double>(BatteryPackACell3) / (1LL << 10);
	temp.batteryPackACell4 = static_cast<double>(BatteryPackACell4) / (1LL << 10);
	temp.batteryPackACell5 = static_cast<double>(BatteryPackACell5) / (1LL << 10);
	temp.batteryPackACell6 = static_cast<double>(BatteryPackACell6) / (1LL << 10);
	temp.batteryPackACell7 = static_cast<double>(BatteryPackACell7) / (1LL << 10);
	temp.batteryPackACell8 = static_cast<double>(BatteryPackACell8) / (1LL << 10);
	temp.batteryPackACell9 = static_cast<double>(BatteryPackACell9) / (1LL << 10);
	temp.batteryPackACell10 = static_cast<double>(BatteryPackACell10) / (1LL << 10);

	temp.batteryPackBCell1 = static_cast<double>(BatteryPackBCell1) / (1LL << 10);
	temp.batteryPackBCell2 = static_cast<double>(BatteryPackBCell2) / (1LL << 10);
	temp.batteryPackBCell3 = static_cast<double>(BatteryPackBCell3) / (1LL << 10);
	temp.batteryPackBCell4 = static_cast<double>(BatteryPackBCell4) / (1LL << 10);
	temp.batteryPackBCell5 = static_cast<double>(BatteryPackBCell5) / (1LL << 10);
	temp.batteryPackBCell6 = static_cast<double>(BatteryPackBCell6) / (1LL << 10);
	temp.batteryPackBCell7 = static_cast<double>(BatteryPackBCell7) / (1LL << 10);
	temp.batteryPackBCell8 = static_cast<double>(BatteryPackBCell8) / (1LL << 10);
	temp.batteryPackBCell9 = static_cast<double>(BatteryPackBCell9) / (1LL << 10);
	temp.batteryPackBCell10 = static_cast<double>(BatteryPackBCell10) / (1LL << 10);

	temp.batteryCell1Thermocouple = BatteryCell1Thermocouple;
	temp.batteryCell2Thermocouple = BatteryCell2Thermocouple;
	temp.batteryCell3Thermocouple = BatteryCell3Thermocouple;
	temp.batteryCell4Thermocouple = BatteryCell4Thermocouple;
	temp.batteryCell5Thermocouple = BatteryCell5Thermocouple;
	temp.batteryCell6Thermocouple = BatteryCell6Thermocouple;
	temp.batteryCell7Thermocouple = BatteryCell7Thermocouple;
	temp.batteryCell8Thermocouple = BatteryCell8Thermocouple;

	temp.systemThermocouple1 = SystemThermocouple1;

	temp_leakDetected = LeakDetected;

	temp_sBCCommsLoss = SBCCommsLoss;


    LeaveMutex(); // CAN Mutex

	lastPosition = temp_cur_position;
	lastMotorStatus = motorStatus;
	
	
	batteryInfoMutex.lock();
	batteryInfo = temp;
	batteryInfoMutex.unlock();

	leakDetected = temp_leakDetected;

	sBCCommsLoss = temp_sBCCommsLoss;


}

BatteryInfo CANIntf::GetBatteryInfo()
{
	BatteryInfo retval;
	
	batteryInfoMutex.lock();
	retval = batteryInfo;
	batteryInfoMutex.unlock();

	return retval;
}

void CANIntf::SetBatteryPosition(unsigned short pos)
{
	if (pos > 100) pos = 100;		// clip value
	if (pos < 0) pos = 0;

	desiredPosition = pos;
}

void CANIntf::SetVehicleCommandData()
{

	int motor_command = 0;


    // setting info to CAN bus
    // general form --> CANBusValue = localValue
    EnterMutex(); // CAN Mutex

	// Enum: {DISABLE = 0, DIRECTION_IN = 1, DIRECTION_OUT = 2} or position 2 - 32
	//MotorCommandPosition = desiredPosition;

    // [0, 100] percent
    //MotorCommandSpeed = 100;

    LeaveMutex(); // CAN Mutex
}

void *CANIntf::UpdateVehicleInfo(void *data) 
{
  CANIntf *instance;
  instance = (CANIntf *)data;

  while (instance->IsInitialized()) 
  {
    instance->SetVehicleCommandData();

    instance->GetVehicleStatusData();

    usleep(updateVehicleInfoSleepTimeInMicroseconds); // time in microseconds
  }
}

// Translate enumerated node state into string
std::string NodeStateToString(e_nodeState state) {
  switch (state) {
  case Initialisation:
    return "Initializing";
  case Disconnected:
    return "Disconnected";
  case Connecting:
    return "Connecting";
  case Stopped:
    return "Stopped";
  case Operational:
    return "Operational";
  case Pre_operational:
    return "Pre-operational";
  case Unknown_state:
  default:
    return "Unknown";
  }
}


void InitNode(CO_Data *d, UNS32 id) 
{
  setNodeId(&CO_VehicleSBC_Data, 0x03);

  setState(&CO_VehicleSBC_Data, Initialisation);
}

void Callback_Initialisation(CO_Data *d) 
{
  ROS_ERROR("CANopenInterface::Callback_Initialisation - SBC Entering Initialisation State");
}

void Callback_PreOperational(CO_Data *d) 
{
  ROS_ERROR("CANopenInterface::Callback_PreOperational - SBC Entering Pre-operational State");
  setState(&CO_VehicleSBC_Data, Operational);
}

void Callback_Operational(CO_Data *d) 
{
  ROS_ERROR("CANopenInterface::Callback_Operational - SBC Entering Operational State");
}

void Callback_SlaveStateChange(CO_Data *d, UNS8 nodeId, e_nodeState newNodeState) 
{
  ROS_INFO("CANopenInterface::Callback_SlaveStateChange - Node 0x%x entered mode %s", nodeId,  NodeStateToString(newNodeState).c_str() );
}

void Exit(CO_Data *d, UNS32 id) { setState(&CO_VehicleSBC_Data, Stopped); }


