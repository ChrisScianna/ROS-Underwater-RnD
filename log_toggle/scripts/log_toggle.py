#!/usr/bin/env python
'''
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
'''

# Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com

import rospy
import subprocess
from mission_manager.msg import ExecuteMission
from mission_manager.msg import ReportExecuteMissionState

NODE_VERSION = "1.01x"
rospy.set_param('/version_numbers/log_toggle', NODE_VERSION)
versionInfo = "Starting log toggle node Version: %s" % NODE_VERSION
rospy.loginfo(versionInfo)

state = False


def startLogging(data):
    global state
    state = True
    print("in start")
    subprocess.call("(/mkIII_ros/catkin_ws/startLogging.sh "+str(data.mission_id)+")", shell=True)
    #rospy.loginfo(rospy.get_caller_id() + "Mission %s started.", str(data.mission_id))


def stopLogging(data):
    global state
    if state:
        if data.execute_mission_state == 2:
            print("killing logging")
            state = False
            subprocess.Popen("(/mkIII_ros/catkin_ws/killLogging.sh)", shell=True)
    else:
        if data.execute_mission_state == 4:
            print("start logging")
            state = True
            subprocess.Popen("(/mkIII_ros/catkin_ws/startLogging.sh " +
                             str(data.mission_id)+")", shell=True)
            print("finished")


def listener():
    rospy.init_node('log_toggle', anonymous=True)
    rospy.Subscriber("/mngr/report_mission_execute_state", ReportExecuteMissionState, stopLogging)
    #rospy.Subscriber("/mngr/execute_mission", ExecuteMission, startLogging)
    rospy.spin()


if __name__ == '__main__':
    listener()
