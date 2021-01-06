/*
 * sea_scan_echo_cmds.h
 */

#ifndef _SEA_SCAN_ECHO_CMDS_H_
#define _SEA_SCAN_ECHO_CMDS_H_

#include <string>
#include <vector>

#include <iostream>
#include <sstream>
#include <iomanip>      // for std::setfill

namespace qna
{
namespace sensor
{

const char START_OF_STRING = '$';
const std::string MESSAGE_ID = "MSALT";
const char CHECKSUM_DELIMITER = '*';
const std::string SENTENCE_TERMINATOR = "\n\r";

const std::string comma_delim = ",";
const std::string star_delim = "*";

const std::string BOOT_CMD_STR = "BOOT";
const std::string COM_CMD_STR = "COM";
const std::string FILTER_CMD_STR = "FILTER";
const std::string INFO_CMD_STR = "INFO";
const std::string LOCKOUT_CMD_STR = "LOCKOUT";
const std::string MARCO_CMD_STR = "MARCO";
const std::string RANGE_CMD_STR = "RANGE";
const std::string RESET_CMD_STR = "RESET";
const std::string SOS_CMD_STR = "SOS";
const std::string TRIGGER_CMD_STR = "TRIGGER";
const std::string THRESHOLD_CMD_STR = "THRESHOLD";
const std::string TX_CMD_STR = "TX";

const std::string ACK_MSG = ",ACK";     // added the comma because when parsing "ACK" can be found in "NACK"
const std::string NACK_MSG = ",NACK";
const std::string POLO_MSG = "POLO";
const std::string INFO_MSG = "INFO";
const std::string DATA_MSG = "DATA";
const std::string IMAGE_MSG = "IMAGE";


class SeaScanEchoSensorMsgs
{
public:
    SeaScanEchoSensorMsgs();
    virtual ~SeaScanEchoSensorMsgs();

    std::string GetBootCmd();
    std::string GetComCmd(unsigned int baudrate);
    std::string GetFilterCmd(unsigned short filter_length);
    std::string GetInfoCmd();
    std::string GetLockoutCmd(float lockout_range);
    std::string GetMarcoCmd();
    std::string GetRangeCmd(float listening_range);
    std::string GetResetCmd();
    std::string GetSosCmd(float speed_of_sound);
    std::string GetTriggerCmd(unsigned short trigger);
    std::string GetThresholdCmd(unsigned int detection_threshold);
    std::string GetTxCmd(bool enabled);

    std::string GetResponseMsgString(std::string msg);

    std::string ComputeChecksum(std::string msg);

    void GetDeviceInfoFromMsg(std::string, unsigned short &majvers, unsigned short &minvers, unsigned short &betavers, std::string &serialnum );
    void GetAltimeterDataFromMsg(std::string msg, float &filtered_echo_range, float &unfiltered_echo_range );
    void GetSonarImageFromMsg(std::string msg, float &range, std::vector<unsigned short> &samples );

private:

};
}
}

#endif // _SEA_SCAN_ECHO_CMDS_H_
