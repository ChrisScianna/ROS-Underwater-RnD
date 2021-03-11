
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

#include "mission_manager/mission.h"
#include "mission_manager/behavior.h"
using mission_manager::Mission;
using mission_manager::Behavior;

Mission::Mission() : missionState(MissionState::READY), m_current_behavior(NULL), elasped_time(0.0)
{
  current_abort_behavior_id = 0;
  current_behavior_id = 0;
}

Mission::~Mission()
{
  if (m_behaviors.size() > 0)
  {
    for (int x = 0; x < m_behaviors.size(); x++)
    {
      delete m_behaviors[x];
    }
    m_behaviors.clear();
  }

  if (m_aborts.size() > 0)
  {
    for (int x = 0; x < m_aborts.size(); x++)
    {
      delete m_aborts[x];
    }
    m_aborts.clear();
  }

  current_abort_behavior_id = 0;
  current_behavior_id = 0;
}

void Mission::Stop()
{
  if (GetState() == MissionState::EXECUTING)
  {
    SetState(MissionState::STOPPED);

    // Wait for the mission processing thread to finish
    ROS_INFO("Stopping mission processing thread");
    m_threadProcMission->interrupt();  // wake up thread if sleeping
    m_threadProcMission->join();

    // Clean up all of the behavior publishers and messages
    ROS_INFO("Stopping and cleaning up active behaviors");
    Behavior* cur_behavior = getNextBehavior(true);
    do
    {
      cur_behavior->stopBehavior();
      cur_behavior = getNextBehavior();
    }
    while (cur_behavior != NULL);
  }
  else if (GetState() == MissionState::ABORTING)
  {
    SetState(MissionState::STOPPED);

    ROS_INFO("Stopping abort mission thread");
    m_threadAbortMission->interrupt();
    m_threadAbortMission->join();

    // Clean up all of the behavior publishers and messages
    ROS_INFO("Stopping and cleaning up active behaviors");
    Behavior* cur_behavior = getNextAbortBehavior(true);
    do
    {
      cur_behavior->stopBehavior();
      cur_behavior = getNextAbortBehavior();
    }
    while (cur_behavior != NULL);
  }
}

void Mission::addBehavior(Behavior* behavior) { m_behaviors.push_back(behavior); }

Behavior* Mission::getNextBehavior(bool reset)
{
  static bool firsttime = true;
  if (reset || (firsttime == true))
  {
    m_behavior_iterator = m_behaviors.begin();
    firsttime = false;
  }

  if (m_behavior_iterator == m_behaviors.end())
  {
    return NULL;
  }

  return *m_behavior_iterator++;
}

void Mission::addAbortBehavior(Behavior* behavior) { m_aborts.push_back(behavior); }

Behavior* Mission::getNextAbortBehavior(bool reset)
{
  static bool abortfirsttime = true;
  if (reset || (abortfirsttime == true))
  {
    m_abort_behavior_iterator = m_aborts.begin();
    abortfirsttime = false;
  }

  if (m_abort_behavior_iterator == m_aborts.end())
  {
    return NULL;
  }

  return *m_abort_behavior_iterator++;
}

void Mission::ProcessState(const auv_interfaces::StateStamped& data)
{
  if (m_current_behavior == NULL) return;

  if ((GetState() != MissionState::ABORTING) && (GetState() != MissionState::EXECUTING)) return;

  boost::mutex::scoped_lock callback_lock(m_mutCallbacks);
  if (m_current_behavior->checkState(data)) callBackTmr.stop();
  callback_lock.unlock();
}

void Mission::SetState(MissionState state)
{
  boost::mutex::scoped_lock run_lock(m_MissionStateLock);
  missionState = state;
  run_lock.unlock();
}

Mission::MissionState Mission::GetState()
{
  MissionState temp;

  boost::mutex::scoped_lock run_lock(m_MissionStateLock);
  temp = missionState;
  run_lock.unlock();

  return temp;
}

void Mission::AbortMissionWrapper(ros::NodeHandle nh)
{
  node_handle = nh;

  AbortMission();
}

void Mission::ExecuteMission(ros::NodeHandle nh)
{
  if ((GetState() == MissionState::EXECUTING) || (GetState() == MissionState::ABORTING))
  {
    ROS_WARN("Mission is already executing or aborting");
    return;
  }

  SetState(MissionState::EXECUTING);
  node_handle = nh;
  // Start the mission processing thread
  m_threadProcMission = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&Mission::processMission, this)));
}

void Mission::processMission()
{
  ROS_INFO("Processing mission");

  SetState(MissionState::EXECUTING);

  current_behavior_id = 0;
  current_behavior_name = "";

  bool firsttime = true;

  try
  {
    while (1)
    {
      ROS_INFO("Mission Execution Loop");

      if (GetState() != MissionState::EXECUTING) break;

      // cancel the previous behaviors abort timer
      if (callBackTmr.isValid()) callBackTmr.stop();

      // Get the next runtime behavior
      boost::mutex::scoped_lock lock(m_mutCallbacks);
      m_current_behavior = getNextBehavior(firsttime);
      lock.unlock();
      firsttime = false;

      if (m_current_behavior == NULL)
      {
        ROS_INFO("Mission Complete - no more behaviors");
        SetState(MissionState::COMPLETE);
        break;  // no more behaviors
      }

      current_behavior_id++;
      current_behavior_name = m_current_behavior->getXmlTag();

      ROS_INFO("Starting behavior [%s]", m_current_behavior->getXmlTag());
      m_current_behavior->startBehavior();

      // Sleep until we need to execute the behavior
      int success = 0;
      try
      {
        success = m_current_behavior->WaitForExecutionTimeSlot();
      }
      catch (boost::thread_interrupted)
      {
        break;
      }

      if (success == -1)
      {
        ROS_WARN("Aborting mission - WaitForExecutionTimeSlot failed");
        AbortMission();
        break;
      }

      if (true == m_current_behavior->getTimeoutEna())
      {
        int behaviorTime = m_current_behavior->computeExecutionTimeForBehavior();
        if (behaviorTime == -1)
        {
          ROS_WARN("Aborting mission - computeExecutionTimeForBehavior failed");
          AbortMission();
          break;
        }

        ROS_INFO("registering abort callback in %d seconds", behaviorTime);
        callBackTmr = node_handle.createTimer(ros::Duration(behaviorTime),
                                              boost::bind(&Mission::AbortMission, this), true);
        // t.start();
        callBackTmr.start();
      }

      m_current_behavior->ExecuteBehavior(node_handle);

      unsigned int loop_delay_usec = 1000;
      float time_elapsed = 0;
      while ((GetState() == MissionState::EXECUTING) && (!m_current_behavior->getBehaviorDone()))
      {
        usleep(loop_delay_usec);
        int behaviorDuration = m_current_behavior->getBehaviorDuration();
        if (behaviorDuration == 0.0) break;
        if (behaviorDuration > 0.0)
        {
          time_elapsed += loop_delay_usec / 1000000.0;  // add seconds passed
          if (time_elapsed >= behaviorDuration) break;
        }
      }
    }
  }
  catch (...)
  {
    ROS_WARN("Mission Execution failed because of an execption");
    AbortMission();
  }

  ROS_INFO("Leaving ProcessMission");
}

void Mission::AbortMission()
{
  if ((GetState() == MissionState::ABORTING) || (GetState() == MissionState::COMPLETE)) return;

  ROS_WARN("Aborting mission");
  SetState(MissionState::ABORTING);

  m_threadAbortMission = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&Mission::processAbort, this)));
}

void Mission::processAbort()
{
  m_threadProcMission->interrupt();
  m_threadProcMission->join();

  current_abort_behavior_id = 0;
  current_abort_behavior_name = "";

  bool abortfirsttime = true;

  try
  {
    while (1)
    {
      // Get the next abort behavior
      boost::mutex::scoped_lock lock(m_mutCallbacks);
      m_current_behavior = getNextAbortBehavior(abortfirsttime);
      lock.unlock();
      abortfirsttime = false;

      if (m_current_behavior == NULL)
      {
        SetState(MissionState::COMPLETE);
        ROS_INFO("Done with abort behaviors");
        break;  // no more behaviors
      }

      current_abort_behavior_id++;
      current_abort_behavior_name = m_current_behavior->getXmlTag();

      ROS_INFO("Starting behavior [%s]", m_current_behavior->getXmlTag());
      m_current_behavior->startBehavior();

      // Sleep until we need to execute the behavior
      int success = 0;
      try
      {
        success = m_current_behavior->WaitForExecutionTimeSlot();
      }
      catch (boost::thread_interrupted)
      {
        break;
      }

      if (success == -1)
      {
        continue;  // go on to the next abort behavior
      }

      int behaviorTime = m_current_behavior->computeExecutionTimeForBehavior();
      if (behaviorTime == -1)
      {
        continue;  // go on to the next abort behavior
      }

      m_current_behavior->ExecuteBehavior(node_handle);

      unsigned int loop_delay_usec = 1000;
      float time_elapsed = 0;
      while (!m_current_behavior->getBehaviorDone())
      {
        usleep(loop_delay_usec);
        int behaviorDuration = m_current_behavior->getBehaviorDuration();
        if (behaviorDuration == 0.0) break;
        if (behaviorDuration > 0.0)
        {
          time_elapsed += loop_delay_usec / 1000000.0;  // add seconds passed
          if (time_elapsed >= behaviorDuration) break;
        }
      }
    }
  }
  catch (...)
  {
    ROS_WARN("Mission Abort Execution failed because of an execption");
  }

  ROS_INFO("LEaving processAbort");
}
