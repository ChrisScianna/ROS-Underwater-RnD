/*  QinetiQ North America (QNA)
 *  350 Second Avenue
 *  Waltham, MA 02451
 *
 *  Proprietary Licensed Information.
 *  Not to be duplicated, used, or disclosed,
 *  except under terms of the license.
 *
 *  Copyright © 2018 QinetiQ North America  All rights reserved.
 */

#ifndef __C3_PROTOCOL_H__
#define __C3_PROTOCOL_H__

#include <stdint.h>
#include <sys/types.h>

namespace c3_protocol
{
#define NAV_LONG_MSG_LENGTH 90
/* contents of a ixBlue C3 'navigation long' packetconverted
 * into a host data structure for publishing to other nodes.
 */
class nav_long {
 private:
    int parse_success; // flag for signaling parsing failure
 public:
    // constructor/destructor
    nav_long(char *buf, ssize_t nbytes);
    ~nav_long();
    // accessors
    int get_parse_success() { return parse_success; }
    // public struct to allow fast access to data
    // avoids individual access functions
    struct nav_long_data_t {
        uint16_t header;
        uint32_t user_status;
        uint32_t algo_status[2];
        float heading;               // IEEE float 32
        float roll;                  // IEEE float 32
        float pitch;                 // IEEE float 32
        float north_speed;           // IEEE float 32
        float east_speed;            // IEEE float 32
        float vertical_speed;        // IEEE float 32
        int32_t latitude;
        int32_t longitude;
        float altitude;              // IEEE float 32
        uint32_t timestamp;
        float heading_err_sd;        // IEEE float 32
        float roll_err_sd;           // IEEE float 32
        float pitch_err_sd;          // IEEE float 32
        float north_speed_err_sd;    // IEEE float 32
        float east_speed_err_sd;     // IEEE float 32
        float vertical_speed_err_sd; // IEEE float 32
        float latitude_err_sd;       // IEEE float 32
        float longitude_err_sd;      // IEEE float 32
        float altitude_err_sd;       // IEEE float 32
    } nl_data;
    nav_long_data_t get_data() { return nl_data; };
};

} /* end namespace c3_protocol */

#endif /* __C3_PROTOCOL_H__ */
