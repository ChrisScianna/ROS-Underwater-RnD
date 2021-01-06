/*
 * sea_scan_echo_sensor.h
 */

#ifndef _SEA_SCAN_ECHO_SENSOR_H_
#define _SEA_SCAN_ECHO_SENSOR_H_

#include <mutex>
#include <thread>
#include <chrono>

#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#include <ros/ros.h>

#include <sea_scan_echo_sensor/ReportAltimeterRange.h>
#include <sea_scan_echo_sensor/ReportSonarImage.h>
#include <sea_scan_echo_sensor/ReportSonarConfiguration.h>
#include <sea_scan_echo_sensor/SetFilterLength.h>
#include <sea_scan_echo_sensor/SetLockoutRange.h>
#include <sea_scan_echo_sensor/SetMaxListeningRange.h>
#include <sea_scan_echo_sensor/SetSpeedOfSoundInWater.h>
#include <sea_scan_echo_sensor/SetTransmitterEnabled.h>
#include <sea_scan_echo_sensor/SetTriggerMode.h>
#include <sea_scan_echo_sensor/SetBaudrate.h>
#include <sea_scan_echo_sensor/SetDetectionThreshold.h>

#include "SerialPort.h"

#include "sea_scan_echo_msgs.h"

namespace qna
{
namespace sensor
{

class SeaScanEchoSensor
{
public:
    SeaScanEchoSensor(ros::NodeHandle& nodeHandle);
    virtual ~SeaScanEchoSensor();

    void ReportSonarImage();
    void ReportSonarConfiguration();
    void ReportAltimeterRange();


    // Serial port related member data
    std::string             filename;
    FMI::SerialPort::Config serialPortConfig;
    FMI::SerialPort         serialPort;
    bool stopSerialPortThread;
    std::thread serialPortThread;
    void ReadFromSerialPort();

    std::thread processResponseThread;
    void HandleSonarResponseMsg();

private:

    std::string serialPortName;
    bool serialPortOpened;
    void SetupSerialPort();
    void TearDownSerialPort();
    int SendMsg(std::string msg);
    std::string ReceiveMsg();

    void handle_SetFilterLength(const sea_scan_echo_sensor::SetFilterLength::ConstPtr& msg);
    void handle_SetLockoutRange(const sea_scan_echo_sensor::SetLockoutRange::ConstPtr& msg);
    void handle_SetMaxListeningRange(const sea_scan_echo_sensor::SetMaxListeningRange::ConstPtr& msg);
    void handle_SetSpeedOfSoundInWater(const sea_scan_echo_sensor::SetSpeedOfSoundInWater::ConstPtr& msg);
    void handle_SetTransmitterEnabled(const sea_scan_echo_sensor::SetTransmitterEnabled::ConstPtr& msg);
    void handle_SetTriggerMode(const sea_scan_echo_sensor::SetTriggerMode::ConstPtr& msg);
    void handle_SetBaudrate(const sea_scan_echo_sensor::SetBaudrate::ConstPtr& msg);
    void handle_SetDetectionThreshold(const sea_scan_echo_sensor::SetDetectionThreshold::ConstPtr& msg);

    ros::NodeHandle& nodeHandle;

    SeaScanEchoSensorMsgs   sensorMsgs;

    ros::Subscriber subscriber_setFilterLength;
    ros::Subscriber subscriber_setLockoutRange;
    ros::Subscriber subscriber_setMaxListeningRange;
    ros::Subscriber subscriber_setSpeedOfSoundInWater;
    ros::Subscriber subscriber_setTransmitterEnabled;
    ros::Subscriber subscriber_setTriggerMode;
    ros::Subscriber subscriber_setBaudrate;
    ros::Subscriber subscriber_setDetectionThreshold;

    ros::Publisher publisher_reportAltimeterRange;
    ros::Publisher publisher_reportSonarImage;
    ros::Publisher publisher_reportSonarConfiguration;

    bool transmitterEnabled;
    float detectionThreshold;
    unsigned short triggerMode;
    unsigned short triggerModeOnceRateHz;
    float speedOfSoundInWaterMetersPerSecond;
    float maxListeningRangeMeters;
    float lockoutRangeMeters;
    unsigned short filterLength;
    unsigned int baudrate;

    unsigned short sonarMajorSWVersion;
    unsigned short sonarMinorSWVersion;
    unsigned short sonarBetaSWVersion;
    std::string sonarSerialNum;

    std::vector<unsigned short>  sonar_samples;
    float sonar_samples_range;

    float unfilteredRange;
    float filteredRange;

    std::vector<std::string> sonar_msgs; 
    std::mutex sonar_msgs_mutex;

};
}
}

#endif // _SEA_SCAN_ECHO_SENSOR_H_
