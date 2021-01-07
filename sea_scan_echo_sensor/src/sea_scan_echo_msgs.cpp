/*
 * sea_scan_echo_cmds.cpp
 *
 */


#include "sea_scan_echo_msgs.h"

namespace qna
{
namespace sensor
{


SeaScanEchoSensorMsgs::SeaScanEchoSensorMsgs() 
{


}

SeaScanEchoSensorMsgs::~SeaScanEchoSensorMsgs()
{

}

// Sending this command will cause the sonar to transfer to its 
// bootloader application which can be used for firmware upgrades.
std::string SeaScanEchoSensorMsgs::GetBootCmd()
{
    return START_OF_STRING + MESSAGE_ID  + comma_delim + BOOT_CMD_STR + CHECKSUM_DELIMITER + "7D" + SENTENCE_TERMINATOR;
}

// Sending this command causes the sonar to store a new baud rate.
// The new baud rate will not take effect until the sonar is RESET or power cycled.
std::string SeaScanEchoSensorMsgs::GetComCmd(unsigned int baudrate)
{
    if ((baudrate < 4800) || (baudrate > 921600)) return "";

    std::string msg = START_OF_STRING + MESSAGE_ID  + comma_delim + COM_CMD_STR  + comma_delim + std::to_string(baudrate) + CHECKSUM_DELIMITER;

    return (msg + ComputeChecksum(msg) + SENTENCE_TERMINATOR);
}

std::string SeaScanEchoSensorMsgs::GetFilterCmd(unsigned short filter_length)
{
    if ((filter_length < 1) || (filter_length > 32)) return "";

    std::string msg = START_OF_STRING + MESSAGE_ID  + comma_delim + FILTER_CMD_STR  + comma_delim + std::to_string(filter_length) + CHECKSUM_DELIMITER;

    return (msg + ComputeChecksum(msg) + SENTENCE_TERMINATOR);
}

std::string SeaScanEchoSensorMsgs::GetInfoCmd()
{
    return START_OF_STRING + MESSAGE_ID  + comma_delim + INFO_CMD_STR + CHECKSUM_DELIMITER + "65" + SENTENCE_TERMINATOR;
}

std::string SeaScanEchoSensorMsgs::GetLockoutCmd(float lockout_range)
{
    if ((lockout_range < 0.5) || (lockout_range > 119.5)) return "";

    std::string msg = START_OF_STRING + MESSAGE_ID  + comma_delim + LOCKOUT_CMD_STR  + comma_delim + std::to_string(lockout_range) + CHECKSUM_DELIMITER;

    return (msg + ComputeChecksum(msg) + SENTENCE_TERMINATOR);
}

std::string SeaScanEchoSensorMsgs::GetMarcoCmd()
{
    return START_OF_STRING + MESSAGE_ID  + comma_delim + MARCO_CMD_STR + CHECKSUM_DELIMITER + "39" + SENTENCE_TERMINATOR;
}

std::string SeaScanEchoSensorMsgs::GetRangeCmd(float listening_range)
{
    if ((listening_range < 1.0) || (listening_range > 120.0)) return "";

    std::string msg = START_OF_STRING + MESSAGE_ID  + comma_delim + RANGE_CMD_STR  + comma_delim + std::to_string(listening_range) + CHECKSUM_DELIMITER;

    return (msg + ComputeChecksum(msg) + SENTENCE_TERMINATOR);
}

std::string SeaScanEchoSensorMsgs::GetResetCmd()
{
    return START_OF_STRING + MESSAGE_ID  + comma_delim + RESET_CMD_STR + CHECKSUM_DELIMITER + "3E" + SENTENCE_TERMINATOR;
}

std::string SeaScanEchoSensorMsgs::GetSosCmd(float speed_of_sound)
{
    // speed of sound is in meters / second
    if ((speed_of_sound < 1400.0) || (speed_of_sound > 1600.0)) return "";

    std::string msg = START_OF_STRING + MESSAGE_ID  + comma_delim + SOS_CMD_STR  + comma_delim + std::to_string(speed_of_sound) + CHECKSUM_DELIMITER;

    return (msg + ComputeChecksum(msg) + SENTENCE_TERMINATOR);
}

std::string SeaScanEchoSensorMsgs::GetTriggerCmd(unsigned short trigger)
{
    if ((trigger < 0) || (trigger > 3)) return "";

    std::string msg = START_OF_STRING + MESSAGE_ID  + comma_delim + TRIGGER_CMD_STR  + comma_delim + std::to_string(trigger) + CHECKSUM_DELIMITER;

    return (msg + ComputeChecksum(msg) + SENTENCE_TERMINATOR);
}

std::string SeaScanEchoSensorMsgs::GetThresholdCmd(unsigned int detection_threshold)
{
    // the detection threshold is the minimum sonar echo count that the sonar 
    // considers a valid echo range. If no data is above this level the the sonar 
    // will report 0.0 meters as the echo range. The default is 716 (or 35%).

    std::string msg = START_OF_STRING + MESSAGE_ID  + comma_delim + THRESHOLD_CMD_STR  + comma_delim + std::to_string(detection_threshold) + CHECKSUM_DELIMITER;

    return (msg + ComputeChecksum(msg) + SENTENCE_TERMINATOR);
}

std::string SeaScanEchoSensorMsgs::GetTxCmd(bool enabled)
{
    std::string msg = START_OF_STRING + MESSAGE_ID  + comma_delim + TX_CMD_STR  + comma_delim + (enabled ? "1" : "0") + CHECKSUM_DELIMITER;

    return (msg + ComputeChecksum(msg) + SENTENCE_TERMINATOR);
}

std::string SeaScanEchoSensorMsgs::GetResponseMsgString(std::string msg)
{

    unsigned first_delim_pos = msg.find(comma_delim);
    unsigned end_pos_of_first_delim = first_delim_pos + comma_delim.length();
    unsigned last_delim_pos = msg.find(star_delim, end_pos_of_first_delim);
    if (last_delim_pos == std::string::npos)
    {
        last_delim_pos = msg.find(comma_delim, end_pos_of_first_delim);
    }

    return msg.substr(end_pos_of_first_delim, last_delim_pos - end_pos_of_first_delim);
}

std::string SeaScanEchoSensorMsgs::ComputeChecksum(std::string msg)
{
    unsigned short crc = 0;
    
    for (size_t i = 0 ; i < msg.size(); i++)
    {
        if ((msg[i] != '$') && (msg[i] != '*'))
            crc ^= msg[i];
    }

    crc &= 0xff;

    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << crc;

    return ss.str();
}

void SeaScanEchoSensorMsgs::GetDeviceInfoFromMsg(std::string msg, unsigned short &majvers, unsigned short &minvers, unsigned short &betavers, std::string &serialnum )
{
    majvers = 0;
    minvers = 0;
    betavers = 0;

    size_t delim_pos = msg.find(comma_delim);                    // this should get to the comma before the "INFO" string
    delim_pos = msg.find(comma_delim, delim_pos+1);              // this should get us to the comma before the major version number
    size_t next_delim_pos = msg.find(comma_delim, delim_pos+1);  // this should get us to the comma after the major version number
    majvers = std::atoi((msg.substr(delim_pos+1, (next_delim_pos-delim_pos-1))).c_str());

    delim_pos = next_delim_pos;
    next_delim_pos = msg.find(comma_delim, delim_pos+1);
    minvers = std::atoi((msg.substr(delim_pos+1, (next_delim_pos-delim_pos-1))).c_str());

    delim_pos = next_delim_pos;
    next_delim_pos = msg.find(comma_delim, delim_pos+1);
    betavers = std::atoi((msg.substr(delim_pos+1, (next_delim_pos-delim_pos-1))).c_str());

    delim_pos = next_delim_pos;
    next_delim_pos = msg.find(star_delim, delim_pos+1);
    serialnum = msg.substr(delim_pos+1, (next_delim_pos-delim_pos-1));
}

void SeaScanEchoSensorMsgs::GetAltimeterDataFromMsg(std::string msg, float &filtered_echo_range, float &unfiltered_echo_range )
{
    filtered_echo_range = 0.0;
    unfiltered_echo_range = 0.0;

    size_t delim_pos = msg.find(comma_delim);                       // this should get to the comma before the "DATA" string
    delim_pos = msg.find(comma_delim, delim_pos+1);                 // this should get us to the comma before the filtered echo range
    size_t next_delim_pos = msg.find(comma_delim, delim_pos+1);     // this should get us to the comma before the unfiltered echo range
    filtered_echo_range = std::atof((msg.substr(delim_pos+1, (next_delim_pos-delim_pos-1))).c_str());

    delim_pos = next_delim_pos;
    next_delim_pos = msg.find(star_delim, delim_pos+1);
    unfiltered_echo_range = std::atof((msg.substr(delim_pos+1, (next_delim_pos-delim_pos-1))).c_str());

}

void SeaScanEchoSensorMsgs::GetSonarImageFromMsg(std::string msg, float &range, std::vector<unsigned short> &samples )
{
    range = 0.0;
    unsigned short num_of_samples = 0;

    size_t delim_pos = msg.find(comma_delim);                 // this should get to the comma before the "IMAGE" string
    delim_pos = msg.find(comma_delim, delim_pos+1);                          // this should get us to the comma before the range
    size_t next_delim_pos = msg.find(comma_delim, delim_pos+1);              // this should get us to the comma before the number of samples
    range = std::atof((msg.substr(delim_pos+1, (next_delim_pos-delim_pos-1))).c_str());

    delim_pos = next_delim_pos;
    next_delim_pos = msg.find(comma_delim, delim_pos+1);
    num_of_samples = std::atoi((msg.substr(delim_pos, (next_delim_pos-delim_pos-1))).c_str());

    for (int x = 0; x < num_of_samples; x++)
    {
        delim_pos = next_delim_pos;
        next_delim_pos = msg.find(comma_delim, delim_pos+1);
        if (next_delim_pos == std::string::npos)
        {
            next_delim_pos = msg.find(star_delim, delim_pos+1);
        }
        samples.push_back(std::stoi(msg.substr(delim_pos+1, (next_delim_pos-delim_pos-1)),0, 16));
    }

}

}
}

