#include <ros/ros.h>
#include <algorithm>
#include <stdio.h>
#include <stdint.h>
#include <sstream>
#include <math.h>

#include "jaus_ros_bridge/ActivateManualControl.h"

#include "thruster_control/ReportRPM.h"
#include "thruster_control/SetRPM.h"

#include "fin_control/ReportAngle.h"
#include "fin_control/SetAngle.h"

#include <autopilot/AutoPilotInControl.h>
#include <autopilot/DesiredRoll.h>
#include <autopilot/DesiredPitch.h>
#include <autopilot/DesiredYaw.h>
#include <autopilot/GetCurrentBehaviors.h>
//#include "pid.h"
#include "sensor_msgs/Imu.h"
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include "mission_manager/SetBehavior.h"
#include "mission_manager/AttitudeServo.h"
#include "mission_manager/DepthHeading.h"
#include "mission_manager/AltitudeHeading.h"
#include "mission_manager/FixedRudder.h"
#include "mission_manager/Waypoint.h"

#include "health_monitor/ReportFault.h"
#include "pose_estimator/CorrectedData.h"

#include "mission_manager/LoadMission.h"
#include "mission_manager/ReportLoadMissionState.h"
#include "mission_manager/ExecuteMission.h"
#include "mission_manager/ReportExecuteMissionState.h"
#include "mission_manager/AbortMission.h"
#include "mission_manager/QueryMissions.h"
#include "mission_manager/ReportMissions.h"
#include "mission_manager/RemoveMissions.h"

#include "payload_manager/PayloadCommand.h"

//#include <iridium/IridiumCntl.h>

// Version log
// 1.1 Added version, fixed update depth pid to be 1/10 of pitch
// 1.4x changes for MKIII
#define NODE_VERSION "1.4x"
// WGS84 Parameters

#define WGS84_A         6378137.0               // major axis
#define WGS84_B         6356752.31424518        // minor axis
#define WGS84_F         0.0033528107            // ellipsoid flattening
#define WGS84_E         0.0818191908            // first eccentricity
#define WGS84_EP        0.0820944379            // second eccentricity

 // UTM Parameters
#define UTM_K0          0.9996                  // scale factor
#define UTM_FE          500000.0                // false easting
#define UTM_FN_N        0.0           // false northing, northern hemisphere
#define UTM_FN_S        10000000.0    // false northing, southern hemisphere
#define UTM_E2          (WGS84_E*WGS84_E)       // e^2
#define UTM_E4          (UTM_E2*UTM_E2)         // e^4
#define UTM_E6          (UTM_E4*UTM_E2)         // e^6
#define UTM_EP2         (UTM_E2/(1-UTM_E2))     // e'^2


class MMTestNode 
{
public:
	MMTestNode(ros::NodeHandle &node_handle);
	void Start();
	void Stop();
  
    void loadMission(std::string mission_filename);
    void executeMission(int id);
    void abortMission();
	void queryMissions();
	void removeMissions();
	void sendReportFaultMsg();

    void testMissionLoading();

    void executeDepthHeadingMission();
    void executeAltitudeHeadingMission();
    void executeFixedRudderMission();
//    void executeIridiumMission();
    void executeWaypointMission();
	void executePayloadCommandMission();

	bool StartSatComms();
	bool StopSatComms();
	bool SendMsg(std::vector<char>& msg);  
	int  NumOfMsgsInXmtQueue();

private:	

//    ros::ServiceServer iridiumCntl_srv_;    // from iridium/src/iridium.cpp
//	bool iridiumAction(iridium::IridiumCntl::Request& req, iridium::IridiumCntl::Response& rsp);
//    bool m_iridiumCommsStarted;

	ros::NodeHandle nh;
	ros::Publisher auto_pilot_in_control_pub;

	ros::Publisher fin_control_pub;
	ros::Publisher thruster_pub;


	ros::Publisher pub_corrected_data;

	ros::Publisher battery_info_pub;
	ros::Publisher leak_detected_pub;

	ros::Publisher load_mission_pub;
	ros::Publisher execute_mission_pub;
	ros::Publisher	abort_mission_pub;
	ros::Publisher	query_missions_pub;
	ros::Publisher	remove_missions_pub;
	ros::Publisher	report_fault_pub;

	ros::Subscriber sub_report_mission_load_state;
	ros::Subscriber sub_report_mission_execute_state;
	ros::Subscriber sub_report_missions;

	ros::Subscriber jaus_ros_sub;

	ros::Subscriber thruster_sub;
	ros::Subscriber fin_control_sub;

	ros::Subscriber desiredRoll_sub_;
	ros::Subscriber desiredYaw_sub_;
	ros::Subscriber desiredPitch_sub_;

//	ros::Subscriber correctedData_sub_;

	ros::Subscriber setAttitudeBehavior_sub_;  
	ros::Subscriber setDepthHeadingBehavior_sub_;
	ros::Subscriber setAltitudeHeadingBehavior_sub_;
	ros::Subscriber setFixedRudderBehavior_sub_;
	ros::Subscriber setWaypointBehavior_sub_;  
	ros::Subscriber setBehavior_sub_;
	ros::ServiceServer curBehaviors_srv_;
	ros::Subscriber payloadCommand_sub_;
/*
	control_toolbox::Pid roll_pid_controller;
	control_toolbox::Pid pitch_pid_controller;
	control_toolbox::Pid yaw_pid_controller;
	control_toolbox::Pid depth_pid_controller;
	control_toolbox::Pid altitude_pid_controller;
	control_toolbox::Pid speed_pid_controller;
*/
	bool auto_pilot_in_control;

	double roll_pgain;
	double roll_igain;
	double roll_dgain;

	double pitch_pgain;
	double pitch_igain;
	double pitch_dgain;

	double yaw_pgain;
	double yaw_igain;
	double yaw_dgain;

	double depth_pgain;
	double depth_igain;
	double depth_dgain;

	double altitude_pgain;
	double altitude_igain;
	double altitude_dgain;

	double speed_pgain;
	double speed_igain;
	double speed_dgain;

	double roll_imax;
	double roll_imin;
	double pitch_imax;
	double pitch_imin;
	double yaw_imax;
	double yaw_imin;
	double depth_imax;
	double depth_imin;
	double altitude_imax;
	double altitude_imin;
	double speed_imax;
	double speed_imin;

//	void correctedDataCallback(const pose_estimator::CorrectedData& data);
	void HandleReportRPM(const thruster_control::ReportRPM& data);
	void HandleReportFinAngle(const fin_control::ReportAngle& data);
	void HandleActivateManualControl(const jaus_ros_bridge::ActivateManualControl& data);

	void mixactuators(double roll, double pitch, double yaw);
	double radiansToDegrees(double radians);
	double degreesToRadians(double degrees);
	void latLongtoUTM(double latitude, double longitude , double * ptrNorthing , double * ptrEasting);
	double angle360(double degval);
	double relAng(double xa, double ya, double xb, double yb);

	void testerFunc();
	boost::shared_ptr<boost::thread> m_thread;
    void positionThread();
	boost::shared_ptr<boost::thread> m_pose_thread;

	boost::mutex m_mutex;

	double command_pos;
	double imuRollOffset;
	double imuPitchOffset;
	double imuYawOffset;
	double currentRoll;
	double currentYaw;
	double currentPitch;
	double currentDepth;
	double currentAltitude;
	double currentNorthing;
	double currentEasting;
	double currentSpeed;

	double currentLat;
	double currentLong;

	double currentRollRate;  //x
	double currentYawRate;   //y
	double currentPitchRate; //z

	double currentRPMs;
	std::vector<double> currentFinAngles;

	double desiredRoll;
	double desiredPitch;
	double desiredYaw;
	double desiredDepth;
	double desiredAltitude;
	double desiredRudder;
	double desiredNorthing;
	double desiredEasting;
	double desiredSpeed;

    double desiredLatitude;
    double desiredLongitude;

    double max_thruster_acceleration;  //knot per second.
    int control_loop_rate; //Hz
    double minimalspeed; //knots


	bool fixedRudder;
	bool depthControl;
    bool altitudeControl;
	bool autopilotEnabled;
	double maxCtrlPlaneAngle;
	bool waypointfollowing;

	int messagectr;

	void setBehaviorCallback(const mission_manager::SetBehavior& msg);
	bool getCurrentBehaviors(autopilot::GetCurrentBehaviors::Request& req, autopilot::GetCurrentBehaviors::Response& rsp);

	// mission manager callbacks
    void reportLoadMissionStateCallback(const mission_manager::ReportLoadMissionState& msg);
    void reportExecuteMissionStateCallback(const mission_manager::ReportExecuteMissionState& msg);
    void reportMissionsCallback(const mission_manager::ReportMissions& msg);

	// Behavior callbacks
	void attitudeServoCallback(const mission_manager::AttitudeServo& msg);
	void altitudeHeadingCallback(const mission_manager::AltitudeHeading& msg);
	void depthHeadingCallback(const mission_manager::DepthHeading& msg);
	void fixedRudderCallback(const mission_manager::FixedRudder& msg);
	void waypointCallback(const mission_manager::Waypoint& msg);
	void payloadCommandCallback(const payload_manager::PayloadCommand& msg);

	// Maps and functions to handle the active subscribers and messages
	typedef std::map<std::string, ros::Subscriber *> sub_map_t;
	sub_map_t m_mapBehaviorSubs;
	bool startBehavior(const std::string& xml_tag, const std::string& topic);
	bool stopBehavior(const std::string& xml_tag);

	// Mask that keeps tracks of active behaviors
	std::vector<std::string> cur_behaviors;
	boost::mutex m_behavior_mutex;
};

MMTestNode::MMTestNode(ros::NodeHandle & node_handle) :
nh(node_handle)
{
    auto_pilot_in_control = true;
    messagectr = 0;
    currentRoll = 0;
    currentYaw = 0;
    currentPitch = 0;
    currentSpeed = 0;

    desiredLatitude = 0.0;
    desiredLongitude = 0.0;
    desiredSpeed = 0;
    desiredRoll = 0;
    desiredYaw = 0;
    desiredPitch = 0;
    desiredDepth = 0;
    desiredAltitude = 10000; //don't want it to dive to bottom if no altitude cmd given
    desiredRudder = 0;
    fixedRudder = false;
    depthControl = false;
    altitudeControl = false;
    waypointfollowing = false;

//    m_iridiumCommsStarted = false;

    currentRollRate = 0;  //x
    currentYawRate = 0;   //y
    currentPitchRate = 0; //z 

    currentDepth = 0;
    currentRPMs = 0.0;

    // not sure if there is a bteer way to insert/add 4 elements into this vector.
    currentFinAngles.push_back(0.0);
    currentFinAngles.push_back(0.0);
    currentFinAngles.push_back(0.0);
    currentFinAngles.push_back(0.0);

    autopilotEnabled = true;
    roll_imax=0;
    roll_imin=0;
    pitch_imax=0;
    pitch_imin=0;
    yaw_imax=0;
    yaw_imin=0;
    maxCtrlPlaneAngle = 20.0;	// this value may need to be updated - pjr 7/3/2019

    max_thruster_acceleration = 1.0;  //knot per second.
    control_loop_rate = 25.0; //Hz
    minimalspeed = 3.0; //knots


    nh.getParam("/autopilot_node/max_thruster_acceleration",max_thruster_acceleration);
    nh.getParam("/autopilot_node/control_loop_rate",control_loop_rate);
    nh.getParam("/autopilot_node/minimalspeed",minimalspeed);

    nh.getParam("/autopilot_node/roll_p",roll_pgain);
    nh.getParam("/autopilot_node/roll_i",roll_igain);
    nh.getParam("/autopilot_node/roll_imax",roll_imax);
    nh.getParam("/autopilot_node/roll_imin",roll_imin);
    nh.getParam("/autopilot_node/roll_d",roll_dgain);

    nh.getParam("/autopilot_node/pitch_p",pitch_pgain);
    nh.getParam("/autopilot_node/pitch_i",pitch_igain);
    nh.getParam("/autopilot_node/pitch_imax",pitch_imax);
    nh.getParam("/autopilot_node/pitch_imin",pitch_imin);
    nh.getParam("/autopilot_node/pitch_d",pitch_dgain);

    nh.getParam("/autopilot_node/yaw_p",yaw_pgain);
    nh.getParam("/autopilot_node/yaw_i",yaw_igain);
    nh.getParam("/autopilot_node/yaw_imax",yaw_imax);
    nh.getParam("/autopilot_node/yaw_imin",yaw_imin);
    nh.getParam("/autopilot_node/yaw_d",yaw_dgain);

    nh.getParam("/autopilot_node/depth_p",depth_pgain);
    nh.getParam("/autopilot_node/depth_i",depth_igain);
    nh.getParam("/autopilot_node/depth_imax",depth_imax);
    nh.getParam("/autopilot_node/depth_imin",depth_imin);
    nh.getParam("/autopilot_node/depth_d",depth_dgain);
 
    nh.getParam("/autopilot_node/altitude_p",altitude_pgain);
    nh.getParam("/autopilot_node/altitude_i",altitude_igain);
    nh.getParam("/autopilot_node/altitude_imax",altitude_imax);
    nh.getParam("/autopilot_node/altitude_imin",altitude_imin);
    nh.getParam("/autopilot_node/altitude_d",altitude_dgain);


    nh.getParam("/autopilot_node/speed_p",speed_pgain);
    nh.getParam("/autopilot_node/speed_i",speed_igain);
    nh.getParam("/autopilot_node/speed_imax",speed_imax);
    nh.getParam("/autopilot_node/speed_imin",speed_imin);
    nh.getParam("/autopilot_node/speed_d",speed_dgain);
 
    nh.getParam("/autopilot_node/max_ctrl_plane_angle",maxCtrlPlaneAngle);
/*
    roll_pid_controller.initPid(roll_pgain,roll_igain,roll_dgain,roll_imax,roll_imin);
    pitch_pid_controller.initPid(pitch_pgain,pitch_igain,pitch_dgain,pitch_imax,pitch_imin);
    yaw_pid_controller.initPid(yaw_pgain,yaw_igain,yaw_dgain,yaw_imax,yaw_imin);
    depth_pid_controller.initPid(depth_pgain,depth_igain,depth_dgain,depth_imax,depth_imin);
    altitude_pid_controller.initPid(altitude_pgain,altitude_igain,altitude_dgain,altitude_imax,altitude_imin);
    speed_pid_controller.initPid(speed_pgain,speed_igain,speed_dgain,speed_imax,speed_imin);
*/
    imuRollOffset = 0;
    imuPitchOffset = 0;
    imuYawOffset = 0;

    nh.getParam("/autopilot_node/imu_roll_offset",imuRollOffset);
    nh.getParam("/autopilot_node/imu_pitch_offset",imuPitchOffset);
    nh.getParam("/autopilot_node/imu_yaw_offset",imuYawOffset);

    nh.getParam("/autopilot_node/desired_roll",desiredRoll);
    nh.getParam("/autopilot_node/desired_pitch",desiredPitch);
    nh.getParam("/autopilot_node/desired_yaw",desiredYaw);
    nh.getParam("/autopilot_node/depth_control_enabled",depthControl);
    nh.getParam("/autopilot_node/desired_depth",desiredDepth);


//    jaus_ros_sub = nh.subscribe("/jaus_ros_bridge/activate_manual_control",1,&AutoPilotNode::HandleActivateManualControl, this);

//    thruster_sub = nh.subscribe("/thruster_control/report_rpm", 1, &AutoPilotNode::HandleReportRPM, this);
//    fin_control_sub = nh.subscribe("/fin_control/report_angle", 10, &AutoPilotNode::HandleReportFinAngle, this);

//    correctedData_sub_ = nh.subscribe("pose/corrected_data", 100, &MMTestNode::correctedDataCallback, this);

    pub_corrected_data = nh.advertise<pose_estimator::CorrectedData>("/pose/corrected_data", 1);


    load_mission_pub = nh.advertise<mission_manager::LoadMission>("mngr/load_mission", 1);
    execute_mission_pub = nh.advertise<mission_manager::ExecuteMission>("mngr/execute_mission", 1);
    abort_mission_pub = nh.advertise<mission_manager::AbortMission>("mngr/abort_mission", 1);
	query_missions_pub = nh.advertise<mission_manager::QueryMissions>("mngr/query_missions", 1);
	remove_missions_pub = nh.advertise<mission_manager::RemoveMissions>("mngr/remove_missions", 1);
	report_fault_pub = nh.advertise<health_monitor::ReportFault>("/health_monitor_node/report_fault", 1);

    sub_report_mission_load_state = nh.subscribe("/mngr/report_mission_load_state", 10, &MMTestNode::reportLoadMissionStateCallback, this);
    sub_report_mission_execute_state = nh.subscribe("/mngr/report_mission_execute_state", 10, &MMTestNode::reportExecuteMissionStateCallback, this);
    sub_report_missions = nh.subscribe("/mngr/report_missions", 10, &MMTestNode::reportMissionsCallback, this);
    
    thruster_pub = nh.advertise<thruster_control::SetRPM>("thruster_control/set_rpm", 1);
    fin_control_pub = nh.advertise<fin_control::SetAngle>("fin_control/set_angle", 10);
    auto_pilot_in_control_pub = nh.advertise<autopilot::AutoPilotInControl>("autopilot/auto_pilot_in_control",1);
    
    //tjw added 
    setAttitudeBehavior_sub_ = nh.subscribe("/mngr/attitude_servo", 10, &MMTestNode::attitudeServoCallback, this);
    setDepthHeadingBehavior_sub_ = nh.subscribe("/mngr/depth_heading", 10, &MMTestNode::depthHeadingCallback, this);
    setAltitudeHeadingBehavior_sub_ = nh.subscribe("/mngr/altitude_heading", 10, &MMTestNode::altitudeHeadingCallback, this);
    setFixedRudderBehavior_sub_ = nh.subscribe("/mngr/fixed_rudder", 10, &MMTestNode::fixedRudderCallback, this);
    setWaypointBehavior_sub_ = nh.subscribe("/mngr/waypoint", 10, &MMTestNode::waypointCallback, this);
  
    payloadCommand_sub_ = nh.subscribe("/payload_manager/command", 10, &MMTestNode::payloadCommandCallback, this);

//  	iridiumCntl_srv_ = nh.advertiseService("/iridium/iridium_cntl", &MMTestNode::iridiumAction, this);
 
   //tjw took out next line
   //setBehavior_sub_ = nh.subscribe("mngr/set_behavior", 10, &AutoPilotNode::setBehaviorCallback, this);

	//curBehaviors_srv_ = nh.advertiseService("current_behaviors", &AutoPilotNode::getCurrentBehaviors, this);

 //  latLongtoUTM(42.3601, -71.0589);
}

void MMTestNode::Start()
{
    assert(!m_thread);
    autopilotEnabled = true;
    m_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&MMTestNode::testerFunc, this)));
    m_pose_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&MMTestNode::positionThread, this)));
}

void MMTestNode::Stop() 
{

    autopilotEnabled = false;
    m_thread->join();
    m_pose_thread->join();

    // Stop all behavior subscribers
    sub_map_t::iterator sub_iter = m_mapBehaviorSubs.begin();
    while (sub_iter != m_mapBehaviorSubs.end()) 
    {
        stopBehavior(sub_iter->first);
        ++sub_iter;
    }

    m_mapBehaviorSubs.clear();

    cur_behaviors.clear();
}

/*
bool MMTestNode::iridiumAction(iridium::IridiumCntl::Request& req, iridium::IridiumCntl::Response& rsp)
{
//	std::stringstream ss;
       boost::mutex::scoped_lock lock(m_behavior_mutex);

//	std::copy(cur_behaviors.begin(), cur_behaviors.end(), std::ostream_iterator<std::string>(ss, ","));

//	rsp.cur_behaviors = ss.str();	
       ROS_INFO("got action request");

       ROS_INFO("request action '%s'",req.Action.c_str());  
       ROS_INFO("request msg '%s'",req.Msg.c_str());
       if(req.Action.compare("start_comms")== 0)
       {
          ROS_INFO("got start comms");
          rsp.Success =  StartSatComms();
       }
       else if(req.Action.compare("stop_comms")== 0)
       {
          ROS_INFO("got stop comms");
          rsp.Success = StopSatComms();
       }
       else if(req.Action.compare("send_msg")== 0)
       {
          std::vector<char> charVec;	      
          charVec.assign(req.Msg.c_str(),req.Msg.c_str()+strlen(req.Msg.c_str()));
          
          ROS_INFO("got send msg");
          rsp.Success = SendMsg(charVec);
       }
       else if(req.Action.compare("status")== 0)
       {
          ROS_INFO("got send status");
          
          rsp.NumOfMsgsInXmtQueue = NumOfMsgsInXmtQueue();
          rsp.Success = (bool) rsp.NumOfMsgsInXmtQueue;
          return true;
       }
       else
       {
          ROS_INFO("unsupported action");
          rsp.Success = false;
       }      

        //we will load up status for every command
        rsp.NumOfMsgsInXmtQueue = NumOfMsgsInXmtQueue();
      
	return true;
}

int MMTestNode::NumOfMsgsInXmtQueue()
{
    return 11;
}

bool MMTestNode::SendMsg(std::vector<char>& msg)
{

	return true;
}

bool MMTestNode::StartSatComms()
{

	m_iridiumCommsStarted = true;
   	return true;
}

bool MMTestNode::StopSatComms() 
{

	m_iridiumCommsStarted = false;
   	return true;
}
*/

//---------------------------------------------------------------
//-------------------------------------------------------------
// This procedure is part of IvP Helm Core Libs  

//* IvP Helm Core Libs is free software: you can redistribute it  */
/* and/or modify it under the terms of the Lesser GNU General    */
/* Public License as published by the Free Software Foundation,  */
/* either version 3 of the License, or (at your option) any      */
/* later version.*/

// Procedure: angle360
//   Purpose: Convert angle to be strictly in the rang [0, 360).

double MMTestNode::angle360(double degval)
{
  while(degval >= 360.0)
    degval -= 360.0;
  while(degval < 0.0)
    degval += 360.0;
  return(degval);
}

//-------------------------------------------------------------
// This procedure is part of IvP Helm Core Libs  

//* IvP Helm Core Libs is free software: you can redistribute it  */
/* and/or modify it under the terms of the Lesser GNU General    */
/* Public License as published by the Free Software Foundation,  */
/* either version 3 of the License, or (at your option) any      */
/* later version.
                                                */
// Procedure: relAng
//   Purpose: Returns relative angle of pt B to pt A. Treats A
//            as the center.
//
//                   0
//                   |
//                   |
//         -90 ----- A ----- 90      
//                   |
//                   |
//                  180

double MMTestNode::relAng(double xa, double ya, double xb, double yb)
{ 
  if((xa==xb)&&(ya==yb))
    return(0);

  double w   = 0;
  double sop = 0;

  if(xa < xb) {
    if(ya==yb)  
      return(90.0);
    else
      w = 90.0;
  }
  else if(xa > xb) {
    if(ya==yb)  
      return(-90.0);
    else
      w = 270.0;
  }

  if(ya < yb) {
    if(xa == xb) 
      return(0.0);
    if(xb > xa) 
      sop = -1.0;
    else 
      sop =  1.0;
  }
  else if(yb < ya) {
    if(xa == xb) 
      return(180);
    if(xb >  xa) 
      sop =  1.0;
    else 
      sop = -1.0;
  }

  double ydiff = yb-ya;
  double xdiff = xb-xa;
  if(ydiff<0) ydiff = ydiff * -1.0;
  if(xdiff<0) xdiff = xdiff * -1.0;

  double avalPI = atan(ydiff/xdiff);
  double avalDG = radiansToDegrees(avalPI);
  double retVal = (avalDG * sop) + w;

  //retVal = angle360(retVal);
  retVal = fmod((retVal+180.0),360.0)-180.0; 

  return(retVal);
}

void MMTestNode::latLongtoUTM(double latitude, double longitude , double * ptrNorthing , double * ptrEasting)
{

//void fromMsg(const geographic_msgs::GeoPoint &from, UTMPoint &to)


     int zone;
     double Lat = latitude;
     double Long = longitude;
     double easting;
     double northing;
     double a = WGS84_A;
     double eccSquared = UTM_E2;
     double k0 = UTM_K0;

     double LongOrigin;
     double eccPrimeSquared;
     double N, T, C, A, M;

ROS_INFO("Lat: [%f]  Long: [%f]", Lat, Long);
           

     // Make sure the longitude is between -180.00 .. 179.9
     // (JOQ: this is broken for Long < -180, do a real normalize)

     double LongTemp = (Long+180)-int((Long+180)/360)*360-180;
     double LatRad = degreesToRadians(Lat);
     double LongRad = degreesToRadians(LongTemp);
     double LongOriginRad;

 ROS_INFO("LongTemp: [%f]", LongTemp);

     zone = int((LongTemp + 180)/6) + 1;

     if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
      zone = 32;

   

     // Special zones for Svalbard

     if( Lat >= 72.0 && Lat < 84.0 ) 
       {
         if(      LongTemp >= 0.0  && LongTemp <  9.0 ) zone = 31;
         else if( LongTemp >= 9.0  && LongTemp < 21.0 ) zone = 33;
         else if( LongTemp >= 21.0 && LongTemp < 33.0 ) zone = 35;
         else if( LongTemp >= 33.0 && LongTemp < 42.0 ) zone = 37;
       }

     // +3 puts origin in middle of zone

     LongOrigin = (zone - 1)*6 - 180 + 3; 
     LongOriginRad = degreesToRadians(LongOrigin);

  
     // compute the UTM band from the latitude
    // to.band = UTMBand(Lat, LongTemp);
#if 0
     if (to.band == ' ')
       throw std::range_error;
#endif


     eccPrimeSquared = (eccSquared)/(1-eccSquared);
     N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
     T = tan(LatRad)*tan(LatRad);
     C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
     A = cos(LatRad)*(LongRad-LongOriginRad);

   
     M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
             - 5*eccSquared*eccSquared*eccSquared/256) * LatRad 
            - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
               + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
            + (15*eccSquared*eccSquared/256
               + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
            - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

     easting = (double)
       (k0*N*(A+(1-T+C)*A*A*A/6
              + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
        + 500000.0);

     northing = (double)
       (k0*(M+N*tan(LatRad)
            *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
              + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

     if(Lat < 0)
       {
         //10000000 meter offset for southern hemisphere
         northing += 10000000.0;
       }

  ROS_INFO("easting: [%f]  northing: [%f] zone: [%d]",easting,northing,zone);

  *ptrNorthing = northing;
  *ptrEasting = easting;


}

double MMTestNode::radiansToDegrees(double radians)
{
   return (radians*(180.0/M_PI));
}

double MMTestNode::degreesToRadians(double degrees)
{
  return((degrees/180.0) * M_PI);
}

void MMTestNode::positionThread()
{
    pose_estimator::CorrectedData msg;

    while (1)
    {
        msg.position.latitude = desiredLatitude;
        msg.position.longitude = desiredLongitude;

        msg.depth = desiredDepth;
        msg.altitude = desiredAltitude;

        msg.speed = desiredSpeed;

        msg.rpy_ang.x = desiredRoll;
        msg.rpy_ang.y = desiredPitch;
        msg.rpy_ang.z = desiredYaw;

        pub_corrected_data.publish(msg);

        usleep(250000);
    }
}

/*
void  MMTestNode::correctedDataCallback(const pose_estimator::CorrectedData& data)
{
//    ROS_INFO("depth from pose: [%f]",data.depth);
    boost::mutex::scoped_lock lock(m_mutex);

    currentDepth = data.depth;
    currentAltitude = data.altitude;

    currentRoll = data.rpy_ang[data.ROLL];

    currentPitch = data.rpy_ang[data.PITCH];

    currentYaw = data.rpy_ang[data.YAW];
    currentYaw = fmod((currentYaw+180.0),360.0)-180.0; // translate from INS 0-360 to +/-180

    currentRollRate = data.rpy_rate[data.ROLL];
    currentPitchRate = data.rpy_rate[data.PITCH];
    currentYawRate = data.rpy_rate[data.YAW];

    currentLat = data.latitude;
    currentLong = data.longitude;
    latLongtoUTM(currentLat, currentLong , &currentNorthing , &currentEasting);
    
    currentSpeed = data.speed;
}
*/

void MMTestNode::HandleReportRPM(const thruster_control::ReportRPM& data)
{
	currentRPMs = data.rpms;
}

void MMTestNode::HandleReportFinAngle(const fin_control::ReportAngle& data)
{
	currentFinAngles.at(data.ID) = data.angle_in_radians;
}


void MMTestNode::HandleActivateManualControl(const jaus_ros_bridge::ActivateManualControl& data)
{
	
}

/*
Definitions:

+ Sternplane  = ds = pitch nose down (all trailing edges down)     controlls pitch
+ Rudder        = dr = turn to port  (all trailing edges to port)  controls yaw
+ Aileron        = da = port side down                             controls roll

View looking forward (from aft of vehicle) with X fin configuration
Fin 1 = d1 = bottom stbd
Fin 2 = d2 = top stbd
Fin 3 = d3 = bottom port
Fin 4 = d4 = top port

Positive (+) fin rotation defined from right hand rule, thumb pointing away from hull

Equations for each fin angle with a given CMD input

d1 =  ds + dr –da
d2 =  ds – dr – da
d3 = -ds +dr –da
d4 = -ds –dr –da
*/


void MMTestNode::mixactuators(double roll, double pitch, double yaw)
{
 

    double ds = pitch; //sternplane
    double dr = yaw;   //rudder
    double da = roll;  //aileron

    double d1;//Fin 1 bottom stbd
    double d2;//Fin 2 top stb
    double d3;//Fin 3 bottom port
    double d4;//Fin 4 top port
    
    d1 = ds + dr - da;

    d1 = fmod((d1+180.0),360.0)-180.0; // translate from INS 0-360 to +/-180


    if (d1 < (-1.0* maxCtrlPlaneAngle))
      d1 = -1.0* maxCtrlPlaneAngle;
    else if (d1 > maxCtrlPlaneAngle)
      d1 = maxCtrlPlaneAngle;

 
    d2 = ds - dr - da;
   
    d2 = fmod((d2+180.0),360.0)-180.0; // translate from INS 0-360 to +/-180


    if (d2 < (-1.0* maxCtrlPlaneAngle))
      d2 = -1.0* maxCtrlPlaneAngle;
    else if (d1 > maxCtrlPlaneAngle)
      d2 = maxCtrlPlaneAngle;

 

    d3 = -ds + dr - da;

    d3 = fmod((d3+180.0),360.0)-180.0; // translate from INS 0-360 to +/-180

    if (d3 < (-1.0* maxCtrlPlaneAngle))
      d3 = -1.0* maxCtrlPlaneAngle;
    else if (d1 > maxCtrlPlaneAngle)
      d3 = maxCtrlPlaneAngle;



    d4 = -ds -dr - da;

    d4 = fmod((d4+180.0),360.0)-180.0; // translate from INS 0-360 to +/-180


    if (d4 < (-1.0* maxCtrlPlaneAngle))
      d4 = -1.0* maxCtrlPlaneAngle;
    else if (d1 > maxCtrlPlaneAngle)
      d4 = maxCtrlPlaneAngle;



    fin_control::SetAngle setAngleMsg;

    setAngleMsg.ID = 1;
    setAngleMsg.angle_in_radians = degreesToRadians(d1);
    fin_control_pub.publish(setAngleMsg);

    setAngleMsg.ID = 2;
    setAngleMsg.angle_in_radians = degreesToRadians(d2);
    fin_control_pub.publish(setAngleMsg);

    setAngleMsg.ID = 3;
    setAngleMsg.angle_in_radians = degreesToRadians(d3);
    fin_control_pub.publish(setAngleMsg);

    setAngleMsg.ID = 4;
    setAngleMsg.angle_in_radians = degreesToRadians(d4);
    fin_control_pub.publish(setAngleMsg);

    
}

void MMTestNode::loadMission(std::string mission_filename)   
{
    mission_manager::LoadMission msg;

    msg.mission_file_full_path = mission_filename;

    load_mission_pub.publish(msg);
}

void MMTestNode::executeMission(int id)
{
    mission_manager::ExecuteMission msg;

    msg.mission_id = id;

    execute_mission_pub.publish(msg);
}

void MMTestNode::abortMission()
{
    mission_manager::AbortMission msg;

    abort_mission_pub.publish(msg);
}

void MMTestNode::queryMissions()
{
    mission_manager::QueryMissions msg;

	query_missions_pub.publish(msg);
}

void MMTestNode::removeMissions()
{
    mission_manager::RemoveMissions msg;

	remove_missions_pub.publish(msg);
}

void MMTestNode::sendReportFaultMsg()
{
	health_monitor::ReportFault message;

	message.header.stamp = ros::Time::now();
	message.fault_id = health_monitor::ReportFault::BATTERY_LEAK_DETECTED_FAULT_ID;

	report_fault_pub.publish(message);
}

void MMTestNode::reportLoadMissionStateCallback(const mission_manager::ReportLoadMissionState& msg)
{
	ROS_INFO("ReportLoadMissionState Callback. mission_id[%d] load State (1=success, 0=failure) [%d]", msg.mission_id, msg.load_state);
}

void MMTestNode::reportExecuteMissionStateCallback(const mission_manager::ReportExecuteMissionState& msg)
{
	ROS_INFO("ReportExecuteMissionState Callback. mission id[%d], mission state[%d](0=Error, 1=aborting, 2=complete, 3=paused, 4=executing), current behavior id[%d] current behavior name[%s]", msg.mission_id, msg.execute_mission_state, msg.current_behavior_id, msg.current_behavior_name.c_str());
}

void MMTestNode::reportMissionsCallback(const mission_manager::ReportMissions& msg)
{
	ROS_INFO("ReportMissions Callback");

	for (int i = 0; i < msg.missions.size(); ++i)
	{
		const mission_manager::MissionData &mdata = msg.missions[i];
		ROS_INFO("Mission Id:[%d] Description:[%s]", mdata.mission_id, mdata.mission_description.c_str());
	}
}

void MMTestNode::attitudeServoCallback(const mission_manager::AttitudeServo& msg)
{
	ROS_INFO("AtitudeServo Callback");

	fixedRudder = false;

	ROS_INFO("Angle for roll: [%f]",msg.roll);
	desiredRoll = msg.roll;

	ROS_INFO("Angle for pitch: [%f]",msg.pitch);
	desiredPitch = msg.pitch;
	depthControl = false; //disable depth control setting pitch instead

	ROS_INFO("Angle for yaw: [%f]",msg.yaw);
	desiredYaw = msg.yaw;

	ROS_INFO("speed_knots: [%f]",msg.speed_knots);
	desiredSpeed = msg.speed_knots;

}

void MMTestNode::depthHeadingCallback(const mission_manager::DepthHeading& msg)
{
	ROS_INFO("DepthHeading Callback");

      fixedRudder = false;

      ROS_INFO("Depth Heading Message Received setting Angle for roll to 0.");
      desiredRoll = 0;
   
      ROS_INFO("depth: [%f]",msg.depth);
      desiredDepth = msg.depth;
      depthControl = true;  //enabling depth control

      ROS_INFO("Angle for yaw: [%f]",msg.heading);
      desiredYaw = msg.heading;

 
      ROS_INFO("speed_knots: [%f]",msg.speed_knots);
      desiredSpeed = msg.speed_knots;
}


void MMTestNode::altitudeHeadingCallback(const mission_manager::AltitudeHeading& msg)
{
	ROS_INFO("AltitudeHeading Callback");

      fixedRudder = false;

      ROS_INFO("Altitude Heading Message Received setting Angle for roll to 0.");
      desiredRoll = 0;
   
      ROS_INFO("altitude: [%f]",msg.altitude);
      desiredAltitude = msg.altitude;
      depthControl = false;  //enabling depth control

      ROS_INFO("Angle for yaw: [%f]",msg.heading);
      desiredYaw = msg.heading;

 
      ROS_INFO("speed_knots: [%f]",msg.speed_knots);
      desiredSpeed = msg.speed_knots;
}


void MMTestNode::fixedRudderCallback(const mission_manager::FixedRudder& msg)
{
	ROS_INFO("Fixed Rudder Callback");

      fixedRudder = true;
      ROS_INFO("Fixed Rudder Message Received setting Angle for roll to 0.");
      desiredRoll = 0;
   
      ROS_INFO("depth: [%f]",msg.depth);
      desiredDepth = msg.depth;
      depthControl = true;  //enabling depth control

      ROS_INFO("Angle for rudder: [%f]",msg.rudder);
      desiredRudder = msg.rudder;

 
      ROS_INFO("speed_knots: [%f]",msg.speed_knots);
      desiredSpeed = msg.speed_knots;

}

void MMTestNode::payloadCommandCallback(const payload_manager::PayloadCommand& msg)
{    
	ROS_INFO("Payload command Callback");

    ROS_INFO("Payload command recieved: [%s]",msg.command.c_str());


}

void MMTestNode::waypointCallback(const mission_manager::Waypoint& msg)
{    
	ROS_INFO("Waypoint Callback");

    waypointfollowing = true;
    ROS_INFO("Waypoint msg received lat: [%f] long: [%f]",msg.latitude,msg.longitude);
    desiredRoll = 0;

    ROS_INFO("depth: [%f]",msg.depth);
    desiredDepth = msg.depth;
    depthControl = true;  //enabling depth control

    desiredLatitude = msg.latitude;
    desiredLongitude = msg.longitude;

    latLongtoUTM(msg.latitude, msg.longitude , &desiredNorthing, &desiredEasting);

    ROS_INFO("UTM desired Easting: [%f]",desiredEasting);
    ROS_INFO("UTM desired Nothing: [%f]",desiredNorthing);


    ROS_INFO("speed_knots: [%f]",msg.speed_knots);
    desiredSpeed = msg.speed_knots;
}

bool MMTestNode::getCurrentBehaviors(autopilot::GetCurrentBehaviors::Request& req, autopilot::GetCurrentBehaviors::Response& rsp)
{
	std::stringstream ss;
	boost::mutex::scoped_lock lock(m_behavior_mutex);

	std::copy(cur_behaviors.begin(), cur_behaviors.end(), std::ostream_iterator<std::string>(ss, ","));

	rsp.cur_behaviors = ss.str();	

	return true;
}

void MMTestNode::setBehaviorCallback(const mission_manager::SetBehavior& msg)
{
	startBehavior(msg.xml_tag, msg.topic);
}

bool MMTestNode::startBehavior(const std::string& xml_tag, const std::string& topic)
{
	ros::Subscriber *sub = NULL;
        ROS_INFO("BEHAVIOR TOPIC");
        ROS_INFO("%s",topic.c_str());


	std::vector<std::string>::iterator flag = std::find(cur_behaviors.begin(), cur_behaviors.end(), xml_tag);
	if (flag != cur_behaviors.end()) return true; // nothing to do

	// Create a new subscriber and subscribe using an appropriate callback
	if (topic.compare("attitude_servo")==0) {
		sub = new ros::Subscriber();
		*sub = nh.subscribe("/mission_manager/attitude_servo", 10, &MMTestNode::attitudeServoCallback, this);
	} else {
		ROS_ERROR("Unsupported behavior tag: %s", xml_tag.c_str());
		return false;
	}

	// Add the new subscriber to the subscriber map
	m_mapBehaviorSubs[xml_tag] = sub;

	// Update the behavior mask
	boost::mutex::scoped_lock lock(m_behavior_mutex);
	cur_behaviors.push_back(xml_tag);

	return true;
}

bool MMTestNode::stopBehavior(const std::string& xml_tag)
{
	sub_map_t::iterator sub_iter;

	std::vector<std::string>::iterator flag = std::find(cur_behaviors.begin(), cur_behaviors.end(), xml_tag);
	if (flag != cur_behaviors.end()) return true; // nothing to do

	sub_iter = m_mapBehaviorSubs.find(xml_tag);

	if (sub_iter == m_mapBehaviorSubs.end()) {
		ROS_ERROR("Enabled behavior does not have a subscriber!");
		return false;
	}

	// Shutdown the subscriber
	sub_iter->second->shutdown();
	delete sub_iter->second;
	
	// Remove the entry from the subscriber map
	m_mapBehaviorSubs.erase(sub_iter);

	// Update the behavior mask
	boost::mutex::scoped_lock lock(m_behavior_mutex);
	cur_behaviors.erase(flag);

	return true;
}

/*
void MMTestNode::workerFunc()
{
   double rollcmdpos = 0;
   double pitchcmdpos = 0;
   double yawcmdpos = 0;
   double yawError = 0;
   double yawDiff = 0;
   double desired_pitch = 0;
   double speedcmd = 0;
   double last_speedcmd = 0;
   ros::Rate r(control_loop_rate);

   int i = 0;
   while(autopilotEnabled)
   {
      rollcmdpos = roll_pid_controller.updatePid(currentRoll-desiredRoll,currentRollRate,ros::Duration(1.0/control_loop_rate));
      if(depthControl) //out put of depth pid will be input to pitch pid
      {
        //update depth 1/10 of the time
        //if(i == 0)
          desired_pitch = depth_pid_controller.updatePid(currentDepth-desiredDepth,ros::Duration(1.0/control_loop_rate));
 
        //if(i >= 9) 
        //  i=0;
        //else
        //  i++;
      // ROS_INFO("desired pitch: [%f]",desired_pitch);
      }
      else if(altitudeControl) //out put of altitude pid will be input to pitch pid
      {
          desired_pitch = altitude_pid_controller.updatePid(currentAltitude-desiredAltitude,ros::Duration(1.0/control_loop_rate));
      }
      else
        desired_pitch = desiredPitch;

      pitchcmdpos = pitch_pid_controller.updatePid(currentPitch-desired_pitch,currentPitchRate,ros::Duration(1.0/control_loop_rate));

      if(!fixedRudder)
      {
         if(waypointfollowing)
	 {
            desiredYaw = relAng(currentEasting, currentNorthing, desiredEasting, desiredNorthing);
         }
         yawDiff = currentYaw-desiredYaw;
      
         yawError = fmod((yawDiff+180.0),360.0)-180.0; 
         yawcmdpos = yaw_pid_controller.updatePid(yawError,currentYawRate,ros::Duration(1.0/control_loop_rate));
      }
      else  //overiding yaw with fixed rudder
      {
         yawcmdpos = desiredRudder;
      }
      mixactuators(rollcmdpos,pitchcmdpos,yawcmdpos);

      speedcmd = speed_pid_controller.updatePid(currentSpeed-desiredSpeed,ros::Duration(1.0/control_loop_rate));
      if(speedcmd < minimalspeed)
      {
         if(desiredSpeed > minimalspeed)
          speedcmd = minimalspeed;
      }      

      if(fabs(speedcmd - last_speedcmd) > (max_thruster_acceleration/control_loop_rate))
      {
         if(speedcmd > last_speedcmd)
         {
            speedcmd += max_thruster_acceleration/control_loop_rate;
         }
         else
         {
            speedcmd -= max_thruster_acceleration/control_loop_rate;
         }
      }
      
      last_speedcmd = speedcmd;

      // thruster values are in RPMs
      thruster_control::SetRPM setrpm;	

      setrpm.commanded_rpms = speedcmd*303.0; //based on UK analysis of 909rpms is equivalent to 3 knots

      if (setrpm.commanded_rpms > thruster_control::SetRPM::MAX_RPM)
      {
	setrpm.commanded_rpms = thruster_control::SetRPM::MAX_RPM;
        ROS_WARN("shaft_speed too high setting to %d",setrpm.commanded_rpms);
      }
      else if((speedcmd < 909.0)&&(desiredSpeed >= 3.0)) // speed command always maintain atleast 909 rpms to equivalent 3 knots   (10 knot/3030rpm) do not what vehicle thruster to completely 
      {
         setrpm.commanded_rpms = 909.0;
      }
      else 
      {
	setrpm.commanded_rpms = (speedcmd*3030.0)/10.0;
      }
      thruster_pub.publish(setrpm);

      
      //maintain control loop rate
      r.sleep();
   }
}
*/

void MMTestNode::testerFunc()
{
    // sleep for a couple of seconds to let the mission_manager startup
    usleep(2000000);    // 2 seconds
    testMissionLoading();

	queryMissions();
//    executeIridiumMission();
	executePayloadCommandMission();

	usleep(60000000);	// 60 seconds
  	executeWaypointMission();

	usleep(60000000);	// 60 seconds
  	executeWaypointMission();
	usleep(500000);	// .5 second
	sendReportFaultMsg();

	removeMissions();
	queryMissions();

	ROS_INFO("DONE with Tests!!!!!!");
}

void MMTestNode::executeDepthHeadingMission()
{
    executeMission(1);
}

void MMTestNode::executeAltitudeHeadingMission()
{
    executeMission(2);
}

void MMTestNode::executeFixedRudderMission()
{
    executeMission(3);
}

/*
void MMTestNode::executeIridiumMission()
{
    executeMission(5);
}
*/
void MMTestNode::executeWaypointMission()
{
    executeMission(4);
}

void MMTestNode::executePayloadCommandMission()
{
	executeMission(5);
}

void MMTestNode::testMissionLoading()
{
    std::string mission_file;

	// "/home/dev/Perforce/pravish_Ubuntu16_5879/FMI_PST/SeaScout/mkIII_ros/catkin_ws/src/mission_manager/missions/"
	// "/home/ssdev/catkin_ws/src/mission_manager/missions/"
	std::string dir_path = "/home/dev/Perforce/pravish_Ubuntu16_5879/FMI_PST/SeaScout/mkIV_ros/catkin_ws/src/mission_manager/missions/";


    mission_file = "fr2.xml";		//should crash mission manager
    loadMission(mission_file);
    usleep(2000000);    // 2 seconds

    mission_file = "missionfilemissingtest.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds

    mission_file = "depthheadingtest.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds


    mission_file = "altitudeheadingtest.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds


    mission_file = "example.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds


    mission_file = "fixedruddertest.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds

/*
    mission_file = "iridiumtest.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds
*/

    mission_file = "logtest.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds


    mission_file = "ping.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds


    mission_file = "waypointtest.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds

    mission_file = "testpayloadbehavior.xml";
    loadMission(dir_path + mission_file);
    usleep(2000000);    // 2 seconds
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission manager test node");
    ros::NodeHandle nh;

    ROS_INFO("Starting mission manager test node Version: [%s]",NODE_VERSION);
    nh.setParam("/version_numbers/mission_manager_test", NODE_VERSION);

    MMTestNode mmTestNode(nh);
    mmTestNode.Start();

    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    ros::spin();

    mmTestNode.Stop();


    return(0);
}




