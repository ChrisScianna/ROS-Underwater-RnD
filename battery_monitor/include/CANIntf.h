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


#ifndef _CAN_INTERFACE_H
#define _CAN_INTERFACE_H

#include <mutex>

#include <pthread.h>
#include <unistd.h>

#include "canfestival/canfestival.h"

#include "CO_VehicleSBC.h"
#include "BatteryInfo.h"

class CANIntf
{
    public:
        CANIntf();
        ~CANIntf();

        bool Init();

		void SetBatteryPosition(unsigned short pos);

		unsigned short GetLastBatteryPosition(){ return lastPosition; }
		unsigned short GetBatteryPositionStatus(){ return lastMotorStatus; }

		BatteryInfo GetBatteryInfo();
		bool GetLeakDetected() {return leakDetected;}

        bool IsInitialized() {return isInitialized;}
        void SetVehicleCommandData();
        void GetVehicleStatusData();

    protected:
        // Threaded functions:
        pthread_t heartbeatUpdateThread;
        static void* UpdateHeartbeat(void *data);

        pthread_t nodeStatusMonitorThread;
        static void* MonitorNodeStatus(void *data);
        void MonitorHeartbeat();
        unsigned short previousHeartbeat; 

        pthread_t vehicleInfoUpdateThread;
        static void* UpdateVehicleInfo(void *data);

		std::mutex batteryInfoMutex;

    private:
        bool SetupCANBus();
        bool isInitialized;

		unsigned short lastMotorStatus;
		unsigned short lastPosition;
		unsigned short desiredPosition;

    	bool DMCHeartbeatTimeout;

		BatteryInfo batteryInfo;

		bool leakDetected;
		unsigned char systemThermocouple1;
		bool sBCCommsLoss;
};

#endif // _DATA_OBJECT_H
