/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 - 2021, QinetiQ, Inc.
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

/* File generated by gen_cfile.py. Should not be modified. */

#ifndef CO_VEHICLESBC_H
#define CO_VEHICLESBC_H

#include "canfestival/data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 CO_VehicleSBC_valueRangeTest (UNS8 typeValue, void * value);
const indextable * CO_VehicleSBC_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode);

/* Master node data struct */
extern CO_Data CO_VehicleSBC_Data;
extern UNS8 platformState;		/* Mapped at index 0x2000, subindex 0x00*/
extern UNS16 SBCHeartbeat;		/* Mapped at index 0x2001, subindex 0x00*/
extern UNS8 MotorControlState;		/* Mapped at index 0x2002, subindex 0x00*/
extern UNS8 MotorEnableDisable;		/* Mapped at index 0x2006, subindex 0x00*/
extern INTEGER32 MotorVelocityCommand;		/* Mapped at index 0x2008, subindex 0x00*/
extern INTEGER32 MotorVelocityFeedback;		/* Mapped at index 0x200E, subindex 0x00*/
extern INTEGER32 MotorCurrentFeedback;		/* Mapped at index 0x2010, subindex 0x00*/
extern UNS16 BatteryVoltage;		/* Mapped at index 0x2013, subindex 0x00*/
extern UNS8 MotorTemperature;		/* Mapped at index 0x2019, subindex 0x00*/
extern UNS16 SoftwareMajorVersion;		/* Mapped at index 0x2020, subindex 0x00*/
extern UNS16 SoftwareMinorVersion;		/* Mapped at index 0x2021, subindex 0x00*/
extern UNS16 SoftwareMicroVersion;		/* Mapped at index 0x2022, subindex 0x00*/
extern UNS16 SoftwareBuildVersion;		/* Mapped at index 0x2023, subindex 0x00*/
extern UNS8 BootloaderMajorVersion;		/* Mapped at index 0x2024, subindex 0x00*/
extern UNS8 BootloaderCommonVersion;		/* Mapped at index 0x2025, subindex 0x00*/
extern UNS8 BootloaderMinorVersion;		/* Mapped at index 0x2026, subindex 0x00*/
extern UNS8 BootloaderBuildVersion;		/* Mapped at index 0x2027, subindex 0x00*/
extern UNS8 BITStatus;		/* Mapped at index 0x2044, subindex 0x00*/
extern UNS8 platformStateStatus;		/* Mapped at index 0x2051, subindex 0x00*/
extern UNS16 MotorControllerHeartbeat;		/* Mapped at index 0x2060, subindex 0x00*/

#endif // CO_VEHICLESBC_H
