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

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com





#include <iostream>

#include <ros/ros.h>
#include "CANIntf.h"
//#include "CO_VehicleSBC.h"

#define CAN_BUS_NAME "0"
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
motorTimeoutSeconds(0.5),		// default to hald a second or 500 milliseconds
enableCANBusLogging(false)
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
          ROS_ERROR("CANopenInterface::shutdownThread [ERROR]: Cannot shutdown vehicleInfoUpdateThread thread. Error = %d\n", retVal);
  }
}

bool CANIntf::Init()
{
    if (isInitialized == true)
    {
        return true;
    }

    velocity_radsec.Set(0.0);
    velocity_feedback_radsec.Set(0.0);
    motor_tempC.Set(0.0);
	last_set_rpm_time.Set(ros::Time::now().toSec());

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
    ROS_INFO("CANopenInterface::Init [INFO]: Opening CAN port\n");
    canport_ = canOpen(&config, &CO_VehicleSBC_Data);

    if (canport_ == NULL) 
    {
        ROS_ERROR("CANopenInterface::Init [ERROR]: Unable to open CAN port\n");
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
    SBCHeartbeat++;  // part of CO_VehicleSBC.h which is generated code from CanFestival 
    LeaveMutex();
    usleep(updateHeartbeatSleepTimeInMicroseconds); // time in microseconds
  }
}

void CANIntf::MonitorHeartbeat() 
{
    // MotorControllerHeartbeat is defined in CO_VehicleSBC.h which is generated code. It is generated from python script called objdictedit.py
    if(MotorControllerHeartbeat == previousHeartbeat)
    {
        DMCHeartbeatTimeout = true;
    }
    else
    {
        DMCHeartbeatTimeout = false;
    }

    previousHeartbeat = MotorControllerHeartbeat;

}

void *CANIntf::MonitorNodeStatus(void *data) 
{
  CANIntf *instance;
  instance = (CANIntf *)data;
  e_nodeState nodeState;

  while (instance->IsInitialized()) 
  {
    instance->MonitorHeartbeat();
	std::vector<std::string> canNodeList = instance->GetCanNodeIdList();

    EnterMutex();

    //Check if SBC is in operational state
    if(getState(&CO_VehicleSBC_Data) == Operational)
    {
      std::vector<std::string>::iterator it;
      for (it = canNodeList.begin(); it != canNodeList.end(); ++it)
      {
        int nodeHex = std::stoi(*it, 0, 16);

        nodeState = getNodeState(&CO_VehicleSBC_Data, nodeHex);

        if(nodeState == Pre_operational)
        {
          masterSendNMTstateChange(&CO_VehicleSBC_Data, nodeHex, NMT_Start_Node);
        }
      }
    }

    LeaveMutex();


    usleep(monitorNodeStatusSleepTimeInMicroseconds); // time in microseconds
  }
}

void CANIntf::GetVehicleStatusData()
{
    double velfeedback_radpersec = 0.0;	// in rad/sec
    unsigned short tempC = 0;

	// not sure what to do with this information returned from the motorcontroller
    unsigned short swMajorVersion = 0;
    unsigned short swMinorVersion = 0;
    unsigned short swMicroVersion = 0;
    unsigned short swBuildVersion = 0;
    unsigned short platformStatus;
	bool mbpCommsLoss = false;
	bool driveMotorOverTemperature = false;
	unsigned short batteryVoltage_mV = 0;
	int motorCurrentFeedback_mA = 0;

    // getting info from CAN bus
    // general form --> localValue = CANBusValue
    EnterMutex(); // CAN Mutex

    // Convert from Q7.8 format to double, units are radians a second
    velfeedback_radpersec = static_cast<double>(MotorVelocityFeedback) / (1LL << 8);

    // we don't do anything with these values
    swMajorVersion = SoftwareMajorVersion;
    swMinorVersion = SoftwareMinorVersion;
    swMicroVersion = SoftwareMicroVersion;
    swBuildVersion = SoftwareBuildVersion;

    tempC = MotorTemperature;
	
	motorCurrentFeedback_mA = MotorCurrentFeedback;

	// this values data is duplicative of the battery information return in the battery_position_control ROS node
	batteryVoltage_mV = BatteryVoltage;	// in mV

	// We get this when the motor controller has not heard from this ROS Node in a specified amount of time (500ms)
	mbpCommsLoss = (BITStatus & 0x1);  // MBP comm loss is the first bit in the BITStatus field

	// This value returns if the motor temperature is over a defined value. TODO: What should we do if it is? Command 0's?
	driveMotorOverTemperature = ((BITStatus >> 1) & 0x1); // Drive Motor Over Temp is the second bit in the BITStatus field

    // read values from CAN
    // Enum: { DUMMY = 0, RESET = 1, ENABLE = 2, EMERGENCY_STOP = 3 }

    platformStatus = platformStateStatus;

    LeaveMutex(); // CAN Mutex

    // set these values outside of the can mutex because they are mutexed also
    velocity_feedback_radsec.Set(velfeedback_radpersec);
    motor_tempC.Set(tempC);

	if (true == enableCANBusLogging)
		ROS_INFO("{ThrusterData,%lf,%hu,%d,%hu}", velfeedback_radpersec, tempC, motorCurrentFeedback_mA, batteryVoltage_mV);
}


void CANIntf::SetVehicleCommandData()
{

    double vel = velocity_radsec.Get(); // get this outside of the can mutex because it is mutexed also
	double last_time_secs = last_set_rpm_time.Get();
	double now = ros::Time::now().toSec();	

	if ((now-last_time_secs) > GetMotorTiemoutSeconds())
	{
		vel = 0.0;
		ROS_INFO("now[%lf] - lastTime[%lf] = %lf. MotorTimeout[%lf] in seconds", now, last_time_secs, now-last_time_secs, GetMotorTiemoutSeconds());	
	}

    // setting info to CAN bus
    // general form --> CANBusValue = localValue
    EnterMutex(); // CAN Mutex

    // Convert velocities to Q21.8 and publish to CANOpen OD:
    MotorVelocityCommand = static_cast<int32_t>(vel * (1LL << 8));

    // Write values to CAN
    // Enum: { DUMMY = 0, RESET = 1, ENABLE = 2, EMERGENCY_STOP = 3 }
    platformState = 2;
    platformStateStatus = 2;

    // Always keep the drive enabled { DISABLE = 0, ENABLE = 1 }
	if (vel == 0.0)
	{
    	MotorEnableDisable = 0;
	}
	else
	{
    	MotorEnableDisable = 1;
	}

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
  ROS_INFO("CANopenInterface::Callback_Initialisation - SBC Entering Initialisation State\n");
}

void Callback_PreOperational(CO_Data *d) 
{
  ROS_INFO("CANopenInterface::Callback_PreOperational - SBC Entering Pre-operational State\n");
  setState(&CO_VehicleSBC_Data, Operational);
}

void Callback_Operational(CO_Data *d) 
{
  ROS_INFO("CANopenInterface::Callback_Operational - SBC Entering Operational State\n");
}

void Callback_SlaveStateChange(CO_Data *d, UNS8 nodeId, e_nodeState newNodeState) 
{
  ROS_INFO("CANopenInterface::Callback_SlaveStateChange - Node 0x%x entered mode %s", nodeId,  NodeStateToString(newNodeState).c_str() );
}

void Exit(CO_Data *d, UNS32 id) { setState(&CO_VehicleSBC_Data, Stopped); }


