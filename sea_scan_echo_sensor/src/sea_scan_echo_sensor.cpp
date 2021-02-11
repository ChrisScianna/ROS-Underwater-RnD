/*
 * sea_scan_echo_sensor.cpp
 *
 */


#include "sea_scan_echo_sensor.h"
#include <iostream>
#include <vector>
#include <sstream>
namespace qna
{
    namespace sensor
    {

        const double MAX_SONAR_RANGE_METERS = 120.0;      // 50 is the default listening range; choose 0.0 thru 120.0
        const double MAX_ECHO_DETECTION_RANGE = 100.0;
        // This is an array representing the sonar ping rate based on the sonar range setting. The values in the array represent ping rate in Hertz.
        // the array index represents the sonar range in meters. So there may be some duplicate rates. There is a 120 meter max range.
        unsigned short SONAR_PING_RATE[] = {0,60,40,36,33,30,30,29,28,27,26,
                                    26,25,24,23,22,22,21,20,19,18,
                                    18,18,17,17,16,16,16,16,16,15,
                                    15,15,14,14,13,13,13,13,13,12,
                                    12,12,12,12,11,11,11,10,10,9,
                                    9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
                                    9, 9, 9, 9, 8, 8, 8, 8, 8, 8,
                                    8, 8, 8, 8, 7, 7, 7, 7, 7, 7,
                                    7, 7, 7, 7, 6, 6, 6, 6, 6, 6,
                                    6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                    6, 6, 6, 6, 5, 5, 5, 5, 5, 5,
                                    5, 5, 5, 5, 5, 5, 5, 5, 5, 5};

        SeaScanEchoSensor::SeaScanEchoSensor(ros::NodeHandle& nodeHandle) :
            nodeHandle(nodeHandle)
        {
            subscriber_setFilterLength =  nodeHandle.subscribe("/sea_scan_echo_sensor/set_filter_length", 1, &SeaScanEchoSensor::handle_SetFilterLength, this);
            subscriber_setLockoutRange =  nodeHandle.subscribe("/sea_scan_echo_sensor/set_lockout_range", 1, &SeaScanEchoSensor::handle_SetLockoutRange, this);
            subscriber_setMaxListeningRange =  nodeHandle.subscribe("/sea_scan_echo_sensor/set_max_listening_range", 1, &SeaScanEchoSensor::handle_SetMaxListeningRange, this);
            subscriber_setSpeedOfSoundInWater =  nodeHandle.subscribe("/sea_scan_echo_sensor/set_speed_of_sound", 1, &SeaScanEchoSensor::handle_SetSpeedOfSoundInWater, this);
            subscriber_setTransmitterEnabled =  nodeHandle.subscribe("/sea_scan_echo_sensor/set_transmitter_enabled", 1, &SeaScanEchoSensor::handle_SetTransmitterEnabled, this);
            subscriber_setTriggerMode =  nodeHandle.subscribe("/sea_scan_echo_sensor/set_trigger_mode", 1, &SeaScanEchoSensor::handle_SetTriggerMode, this);
            subscriber_setBaudrate =  nodeHandle.subscribe("/sea_scan_echo_sensor/set_baudrate", 1, &SeaScanEchoSensor::handle_SetBaudrate, this);
            subscriber_setDetectionThreshold =  nodeHandle.subscribe("/sea_scan_echo_sensor/set_detection_threshold", 1, &SeaScanEchoSensor::handle_SetDetectionThreshold, this);

            publisher_reportAltimeterRange = nodeHandle.advertise<sea_scan_echo_sensor::ReportAltimeterRange>("/sea_scan_echo_sensor/report_altimeter_range", 1);
            publisher_reportSonarImage = nodeHandle.advertise<sea_scan_echo_sensor::ReportSonarImage>("/sea_scan_echo_sensor/report_sonar_image", 1);
            publisher_reportSonarConfiguration = nodeHandle.advertise<sea_scan_echo_sensor::ReportSonarConfiguration>("/sea_scan_echo_sensor/report_sonar_configuration", 1);


            // getParam does not like unsigned or short typed variables
            int tempBaudRate = 0;
            int tempFilterLength = 0;
            int tempTriggerMode = 0;
            int tempTriggerModeOnceRateHz = 0;

            bool retval = nodeHandle.getParam("/sea_scan_echo_sensor_node/serial_port_baudrate", tempBaudRate);
            if (retval == false)
            {
                ROS_ERROR("Error getting baudrate");
            }
            nodeHandle.getParam("/sea_scan_echo_sensor_node/filter_length", tempFilterLength);
            nodeHandle.getParam("/sea_scan_echo_sensor_node/trigger_mode", tempTriggerMode);
            nodeHandle.getParam("/sea_scan_echo_sensor_node/trigger_mode_once_rate_hz", tempTriggerModeOnceRateHz);
            baudrate = abs(tempBaudRate);
            filterLength = abs(tempFilterLength);
            triggerMode = abs(tempTriggerMode);         // 0 - 3
            triggerModeOnceRateHz = abs(tempTriggerModeOnceRateHz);


            nodeHandle.getParam("/sea_scan_echo_sensor_node/serial_port_name", serialPortName);
            nodeHandle.getParam("/sea_scan_echo_sensor_node/transmitter_enabled", transmitterEnabled);
            nodeHandle.getParam("/sea_scan_echo_sensor_node/detection_threshold", detectionThreshold);
            nodeHandle.getParam("/sea_scan_echo_sensor_node/speed_of_sound_in_water_meters_per_second", speedOfSoundInWaterMetersPerSecond);
            nodeHandle.getParam("/sea_scan_echo_sensor_node/max_listening_range_meters", maxListeningRangeMeters);
            nodeHandle.getParam("/sea_scan_echo_sensor_node/lockout_range_meters", lockoutRangeMeters);


            ROS_INFO("baudrate:[%u]",baudrate);
            ROS_INFO("filterLength:[%u]",filterLength);
            ROS_INFO("trigger mode[%u]", triggerMode);
            ROS_INFO("trigger mode once rate:[%u]hz", triggerModeOnceRateHz);
            ROS_INFO("serial port name:[%s]", serialPortName.c_str());
            ROS_INFO("transmitterEnabled:[%s]", transmitterEnabled ? "true" : "false");
            ROS_INFO("detectionThreshold:[%f]", detectionThreshold);
            ROS_INFO("speedOfSoundInWaterMetersPerSecond:[%f]", speedOfSoundInWaterMetersPerSecond);
            ROS_INFO("maxListeningRangeMeters:[%f]", maxListeningRangeMeters);
            ROS_INFO("lockoutRangeMeters:[%f]", lockoutRangeMeters);


            SetupSerialPort();

            if (serialPortOpened == true)
            {
                ROS_INFO("Serial port %s successfully opened!", serialPortName.c_str() );
                stopSerialPortThread = false;
                serialPortThread = std::thread(&SeaScanEchoSensor::ReadFromSerialPort, this);
                processResponseThread = std::thread(&SeaScanEchoSensor::HandleSonarResponseMsg, this);

                // Need to wait until we get the info information back
                while ( sonarSerialNum.empty() )
                {
                    std::string infoMsg = sensorMsgs.GetInfoCmd();
                    SendMsg(infoMsg);
                    usleep(250000);     // 250000 = 4 times per second
                    ROS_DEBUG("waiting for sonar serial number");
                }

                // send trigger messages to sonar
                // TriggerMode 0=None/Off, 1=Manual (trigger on Sync), 2=Auto (trigger on internal timer), 3=Once (ping Once now)
                std::string echoMsg = sensorMsgs.GetTriggerCmd(triggerMode);
                
                while (true) {
                    SendMsg(echoMsg);
                    std::string response = ReceiveMsg();

                    if (response.compare(ACK_MSG))
                    {
                        break;
                    }
                    else if (response.compare(NACK_MSG))
                    {
                        ROS_WARN("NACK received. Something went wrong with the message.");
                        break;
                    }
                }
                ROS_INFO("Set trigger to automatic");
            }
            else
            {
            }
        }

        SeaScanEchoSensor::~SeaScanEchoSensor()
        {
            stopSerialPortThread = true;
            serialPortThread.join();
            processResponseThread.join();
            TearDownSerialPort();
        }

        // The default baud rate of Sea Scan Echo Altimeter (P/N: 5ECHO1-01) is 38400 bps, 8 bits, no parity, and 1 stop bit. 
        // The default baud rate of Sea Scan Echo Imager (P/N: 5ECHO1-02) is 460800 bps, 8 bits, no parity, and 1 stop bit.
        void SeaScanEchoSensor::SetupSerialPort()
        {

            serialPortOpened = false;

            serialPort.SetName(serialPortName);


            // if Open returns a -1 then something went wrong opening the serial port
            int retval = serialPort.Open();     // if Open is succesful it automatically calls the same method as SetConfig

            if (retval != -1)
            {
                serialPortOpened = true;
            }
            else
            {
                ROS_ERROR("Could not Open the serial port[%s], error=%d", serialPortName.c_str(), retval );
            }

            if (baudrate == 38400)
            {
                serialPortConfig.BaudRate = FMI::SerialPort::Config::BaudRate_T::b38400;
                ROS_INFO("Baud rate set to 38400");
            }
            else
            {
                serialPortConfig.BaudRate = FMI::SerialPort::Config::BaudRate_T::b921600;
                ROS_INFO("Baud rate set to 921600");
            }

            serialPortConfig.Parity = FMI::SerialPort::Config::Parity_T::NOPARITY;
            serialPortConfig.FlowControl = FMI::SerialPort::Config::FlowControl_T::NOFC;
            serialPortConfig.StopBits = 1;
            int config_retval = serialPort.SetConfig(serialPortConfig);
            if (config_retval != 0)
            {
                ROS_ERROR("Could not set the desired configuration. SetConfig returned %d", config_retval);
                return;
            }

        }

        void SeaScanEchoSensor::TearDownSerialPort()
        {
            if (serialPortOpened == true)
            {
                int retval = serialPort.Close();
            }
        }

        int SeaScanEchoSensor::SendMsg(std::string msg)
        {
            int retval = -1;
            if (serialPortOpened == true)
            {
                if (serialPort.SendReady())
                {
                    retval = serialPort.Send(msg.data(), msg.length());
                }
            }
            return retval;
        }

        void SeaScanEchoSensor::HandleSonarResponseMsg()
        {

            while (stopSerialPortThread == false)
            {
                std::string msg = ReceiveMsg();

                if (msg.length() > 0)
                {
                    ROS_DEBUG("Just received msg %s", msg.c_str());

                    std::vector<std::string> filtered_message;
                    std::stringstream s_stream(msg);
                    serialPort.IFlush();
                    while (s_stream.good()) {
                        std::string sub_message;
                        getline(s_stream, sub_message, ',');
                        filtered_message.push_back(sub_message);
                    }
                    if (filtered_message.size() == 0) {
                        ROS_WARN("message does not contain commas");
                        return;

                    }
                    try {
                        AltimeterResponses first_word = altimeter_responses_map.at(filtered_message.at(0));
                    
                        switch (first_word){
                            case AltimeterResponses::ADV:
                                ROS_DEBUG("ADV command found");
                                break;
                            case AltimeterResponses::DTM:
                                ROS_DEBUG("DTM command found");
                                break;
                            case AltimeterResponses::GBS:
                                ROS_DEBUG("GBS command found");
                                break;
                            case AltimeterResponses::GGA:
                                ROS_DEBUG("GGA command found");
                                break;
                            case AltimeterResponses::GNS:
                                ROS_DEBUG("GNS command found");
                                break;
                            case AltimeterResponses::GRS:
                                ROS_DEBUG("GRS command found");
                                break;
                            case AltimeterResponses::GSA:
                                ROS_DEBUG("GSA command found");
                                break;
                            case AltimeterResponses::GST:
                                ROS_DEBUG("GST command found");
                                break;
                            case AltimeterResponses::GSV:
                                ROS_DEBUG("GSV command found");
                                break;
                            case AltimeterResponses::HDT:
                                ROS_DEBUG("HDT command found");
                                break;
                            case AltimeterResponses::LLQ:
                                ROS_DEBUG("LLQ command found");
                                break;
                            case AltimeterResponses::MSALT:
                                {
                                    ROS_DEBUG("MSALT command found");
                                    if (filtered_message.size() < 2) {
                                        ROS_WARN("Filtered message has no arguments after MSALT");
                                        return;
                                    }
                                    std::string matchingPhraseFound("");
                                    for (auto iter = altimeter_sub_responses_map.begin(); iter != altimeter_sub_responses_map.end(); ++iter) {
                                        std::string key = iter->first;
                                        if (filtered_message.at(1).find(key) != std::string::npos) {
                                            matchingPhraseFound = key;
                                            break;
                                        }

                                    }
                                    if (matchingPhraseFound == "") {
                                        ROS_WARN("Unable to find match for response argument %s", filtered_message.at(1).c_str());
                                        return;
                                    }
                                    AltimeterSubResponses second_word = altimeter_sub_responses_map.at(matchingPhraseFound);
                    
                                    switch (second_word){
                                        case AltimeterSubResponses::ACK:
                                            ROS_DEBUG("ACK found as second arg");
                                            break;
                                        case AltimeterSubResponses::DATA:
                                            ROS_DEBUG("DATA found as second arg");
                                            sensorMsgs.GetAltimeterDataFromMsg(msg, unfilteredRange, filteredRange);
                                            ROS_DEBUG("Unfiltered Range %f", unfilteredRange);
                                            ROS_DEBUG("Filtered Range %f", filteredRange);
                                            ReportAltimeterRange();
                                            break;
                                        case AltimeterSubResponses::IMAGE:
                                            ROS_DEBUG("IMAGE found as second arg");
                                            sensorMsgs.GetSonarImageFromMsg(msg, sonar_samples_range, sonar_samples);
                                            ReportSonarImage();
                                            break;
                                        case AltimeterSubResponses::INFO:
                                            ROS_DEBUG("INFO found as second arg");
                                            sensorMsgs.GetDeviceInfoFromMsg(msg, sonarMajorSWVersion, sonarMinorSWVersion, sonarBetaSWVersion, sonarSerialNum);
                                            ROS_DEBUG("Major %d", sonarMajorSWVersion);
                                            ROS_DEBUG("Minor %d", sonarMinorSWVersion);
                                            ROS_DEBUG("Beta %d", sonarBetaSWVersion);
                                            ROS_DEBUG("SerialNum %s", sonarSerialNum.c_str());
                                            ReportSonarConfiguration();
                                            break;
                                        case AltimeterSubResponses::NACK:
                                            ROS_DEBUG("NACK found as second arg");
                                            break;
                                        case AltimeterSubResponses::POLO:
                                            ROS_DEBUG("POLO found as second arg");
                                            break;
                                        case AltimeterSubResponses::TX:
                                            ROS_DEBUG("TX found as second arg");
                                            break;
                                        default:
                                            ROS_WARN("Unable to recognize second word in response. This should not be hit");
                                    }
                                }
                                break;
                            case AltimeterResponses::PFUGDP:
                                ROS_DEBUG("PFUGDP command found");
                                break;
                            case AltimeterResponses::PTNL:
                                ROS_DEBUG("PTNL command found");
                                break;
                            case AltimeterResponses::RMC:
                                ROS_DEBUG("RMC command found");
                                break;
                            case AltimeterResponses::ROT:
                                ROS_DEBUG("ROT command found");
                                break;
                            case AltimeterResponses::VTG:
                                ROS_DEBUG("VTG command found");
                                break;
                            case AltimeterResponses::ZDA:
                                ROS_DEBUG("ZDA command found");
                                break;
                            default:
                                ROS_WARN("Unrecognized first term [%s]",filtered_message.at(0).c_str());
                                break;

                    }
                    } catch (std::out_of_range& ex) {
                        ROS_WARN("Unable to find word matching [%s]", filtered_message.at(0).c_str());
                        
                    }
                }
                else // no message strings to process
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

            }
        }

        std::string SeaScanEchoSensor::ReceiveMsg()
        {    
            std::string retstr = "";        
            sonar_msgs_mutex.lock();   
            int numOfMsgs = sonar_msgs.size();     
            sonar_msgs_mutex.unlock();
            if ( numOfMsgs > 0 )
            {
                sonar_msgs_mutex.lock();
                retstr = sonar_msgs.front();
                sonar_msgs.erase(sonar_msgs.begin());
                sonar_msgs_mutex.unlock();
            }
            //ROS_DEBUG("retstr: %s, numOfMsgs: %d", retstr.c_str(), numOfMsgs);
            return retstr;
        }

        void SeaScanEchoSensor::ReadFromSerialPort()
        {

            ROS_DEBUG("in ReadFromSerialPort");

            serialPort.IFlush();
            serialPort.OFlush();

            std::string partialMsgBuffer;
            while (stopSerialPortThread != true)
            {


                if (serialPort.ReceiveReady(0, 250000) == false)    // loop until there is something to read
                {
                    continue;
                }


                char buf[1052];     // this is to accomodate reading a sonar image. Most messages are much smaller
                memset(buf, 0, 1052);
                int bytesread = serialPort.Receive(buf, 1052);

                if (bytesread != -1)
                {
                    std::string tempstr;
                    tempstr.insert(0, buf, bytesread);
                    partialMsgBuffer.append(tempstr);

                    //ROS_DEBUG("just read %d bytes. string is:%s", bytesread, tempstr.c_str());
                    //ROS_DEBUG("partialMsgBuffer >%s< size is:%d", partialMsgBuffer.c_str(), (int)partialMsgBuffer.size() );

                    std::size_t startMsgPos = partialMsgBuffer.find(START_OF_STRING,0);
                    std::size_t endMsgPos = partialMsgBuffer.find(SENTENCE_TERMINATOR,0);
                    if ((startMsgPos != std::string::npos) && (endMsgPos != std::string::npos) && (partialMsgBuffer.size() >= (endMsgPos + 2 - startMsgPos)))
                    {
                        sonar_msgs_mutex.lock();
                        sonar_msgs.push_back(partialMsgBuffer.substr(startMsgPos, endMsgPos));  // don't include linefeed and carriage return
                        sonar_msgs_mutex.unlock();

                        partialMsgBuffer.erase(startMsgPos, endMsgPos+2);

                        // not sure if we should clear the partialMsgBuffer here
                        // partialMsgBuffer.clear();
                    }

                } // if bytesRead != -1
 
            } // while not stopSerialThread
        }


        void SeaScanEchoSensor::ReportSonarConfiguration()
        {

            sea_scan_echo_sensor::ReportSonarConfiguration message;

            message.stamp = ros::Time::now();
            message.TransmitterEnabled = transmitterEnabled;
            message.DetectionThreshold = detectionThreshold;
            message.TriggerMode = triggerMode;
            message.TriggerModeOnceRateHz = triggerModeOnceRateHz;
            message.SpeedOfSoundInWaterMetersPerSecond = speedOfSoundInWaterMetersPerSecond;
            message.MaxListeningRangeMeters = maxListeningRangeMeters;
            message.LockoutRangeMeters = lockoutRangeMeters;
            message.SonarMajorSWVersion = sonarMajorSWVersion;
            message.SonarMinorSWVersion = sonarMinorSWVersion;
            message.SonarBetaSWVersion = sonarBetaSWVersion;
            message.SonarSerialNum = sonarSerialNum;
            message.FilterLength = filterLength;
            message.Baudrate = baudrate;

            publisher_reportSonarConfiguration.publish(message);
        }

        void SeaScanEchoSensor::ReportAltimeterRange()
        {

            sea_scan_echo_sensor::ReportAltimeterRange message;

            message.stamp = ros::Time::now();
            message.FilteredRange = filteredRange;
            message.UnfilteredRange = unfilteredRange;

            publisher_reportAltimeterRange.publish(message);

        }


        void SeaScanEchoSensor::ReportSonarImage()
        {

            sea_scan_echo_sensor::ReportSonarImage message;

            message.stamp = ros::Time::now();
            message.range = sonar_samples_range;
            message.samples_size = sonar_samples.size();
            for (int x = 0; x < sonar_samples.size(); x++)
            {
                message.samples[x] = sonar_samples.at(x);
            }

            publisher_reportSonarImage.publish(message);

        }

        void SeaScanEchoSensor::handle_SetFilterLength(const sea_scan_echo_sensor::SetFilterLength::ConstPtr& msg)
        {
            std::string echoMsg = sensorMsgs.GetFilterCmd(msg->FilterLength);
            SendMsg(echoMsg);
            std::string response = ReceiveMsg();

            if (response.compare(ACK_MSG))
            {
                // everything is good
                filterLength = msg->FilterLength;
                ReportSonarConfiguration();
            }
            else if (response.compare(NACK_MSG))
            {
                // something went wrong
                ROS_WARN("handle_SetFilterLength - Received NACK");
            }

    
        }

        void SeaScanEchoSensor::handle_SetLockoutRange(const sea_scan_echo_sensor::SetLockoutRange::ConstPtr& msg)
        {
            std::string echoMsg = sensorMsgs.GetLockoutCmd(msg->LockoutRangeMeters);
            SendMsg(echoMsg);
            std::string response = ReceiveMsg();

            if (response.compare(ACK_MSG))
            {
                // everything is good
                lockoutRangeMeters = msg->LockoutRangeMeters;
                ReportSonarConfiguration();
            }
            else if (response.compare(NACK_MSG))
            {
                // something went wrong
                ROS_WARN("handle_SetLockoutRange - Received NACK");
            }
        }

        void SeaScanEchoSensor::handle_SetMaxListeningRange(const sea_scan_echo_sensor::SetMaxListeningRange::ConstPtr& msg)
        {
            std::string echoMsg = sensorMsgs.GetRangeCmd(msg->MaxListeningRangeMeters);
            SendMsg(echoMsg);
            std::string response = ReceiveMsg();

            if (response.compare(ACK_MSG))
            {
                // everything is good
                maxListeningRangeMeters = msg->MaxListeningRangeMeters;
                ReportSonarConfiguration();
            }
            else if (response.compare(NACK_MSG))
            {
                // something went wrong
                ROS_WARN("handle_SetMaxListeningRange - Received NACK");
            }
        }

        void SeaScanEchoSensor::handle_SetSpeedOfSoundInWater(const sea_scan_echo_sensor::SetSpeedOfSoundInWater::ConstPtr& msg)
        {
            std::string echoMsg = sensorMsgs.GetSosCmd(msg->SpeedOfSoundInWaterMetersPerSecond);
            SendMsg(echoMsg);
            std::string response = ReceiveMsg();

            if (response.compare(ACK_MSG))
            {
                // everything is good
                speedOfSoundInWaterMetersPerSecond = msg->SpeedOfSoundInWaterMetersPerSecond;
                ReportSonarConfiguration();
            }
            else if (response.compare(NACK_MSG))
            {
                // something went wrong
                ROS_WARN("handle_SetSpeedOfSoundInWater - Received NACK");
            }

        }

        void SeaScanEchoSensor::handle_SetTransmitterEnabled(const sea_scan_echo_sensor::SetTransmitterEnabled::ConstPtr& msg)
        {
            std::string echoMsg = sensorMsgs.GetTxCmd(msg->TransmitterEnabled);
            SendMsg(echoMsg);
            std::string response = ReceiveMsg();

            if (response.compare(ACK_MSG))
            {
                // everything is good
                transmitterEnabled = msg->TransmitterEnabled;
                ReportSonarConfiguration();
            }
            else if (response.compare(NACK_MSG))
            {
                // something went wrong
                ROS_WARN("handle_SetTransmitterEnabled - Received NACK");
            }
        }

        void SeaScanEchoSensor::handle_SetTriggerMode(const sea_scan_echo_sensor::SetTriggerMode::ConstPtr& msg)
        {
            std::string echoMsg = sensorMsgs.GetTriggerCmd(msg->TriggerMode);
            SendMsg(echoMsg);
            std::string response = ReceiveMsg();

            if (response.compare(ACK_MSG))
            {
                // everything is good
                triggerMode = msg->TriggerMode;
                ReportSonarConfiguration();
            }
            else if (response.compare(NACK_MSG))
            {
                // something went wrong
                ROS_WARN("handle_SetTriggerMode - Received NACK");
            }
        }

        void SeaScanEchoSensor::handle_SetBaudrate(const sea_scan_echo_sensor::SetBaudrate::ConstPtr& msg)
        {
            std::string echoMsg = sensorMsgs.GetComCmd(msg->Baudrate);
            SendMsg(echoMsg);
            std::string response = ReceiveMsg();

            if (response.compare(ACK_MSG))
            {
                // everything is good
                baudrate = msg->Baudrate;
                // I suppose if this command is succesful that we should close and reopen the serial port
                ReportSonarConfiguration();
            }
            else if (response.compare(NACK_MSG))
            {
                // something went wrong
                ROS_WARN("handle_SetBaudrate - Received NACK");
            }
        }

        void SeaScanEchoSensor::handle_SetDetectionThreshold(const sea_scan_echo_sensor::SetDetectionThreshold::ConstPtr& msg)
        {
            std::string echoMsg = sensorMsgs.GetThresholdCmd(msg->DetectionThreshold);
            SendMsg(echoMsg);
            std::string response = ReceiveMsg();

            if (response.compare(ACK_MSG))
            {
                // everything is good
                detectionThreshold = msg->DetectionThreshold;
                ReportSonarConfiguration();
            }
            else if (response.compare(NACK_MSG))
            {
                // something went wrong
                ROS_WARN("handle_SetDetectionThreshold - Received NACK");
            }
        }





    }
}

