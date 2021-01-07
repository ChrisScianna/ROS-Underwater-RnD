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

/* TO-DO:
 * - add error logging
 */

#include <arpa/inet.h>
#include <ros/ros.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "c3_protocol.h"

namespace c3_protocol
{

#define PACKED __attribute__((__packed__))
#define ONE_PI_RADIANS 3.14159265
#define TWO_PI_RADIANS 6.28318531
#define MAX_DEGREES 180
#define HOURS_IN_A_DAY 24
#define MINUTES_IN_AN_HOUR 60
#define SECONDS_IN_A_MINUTE 60

/* Contents of a ixBlue c3 'navigation long' packet
   See pages 370 and 371 (numbered) of the INS- Interface Library
 */
struct PACKED nav_long_pkt {
    uint16_t header;
    uint32_t user_status;
    uint32_t algo_status[2];
    uint32_t heading;               // IEEE float 32
    uint32_t roll;                  // IEEE float 32
    uint32_t pitch;                 // IEEE float 32
    uint32_t north_speed;           // IEEE float 32
    uint32_t east_speed;            // IEEE float 32
    uint32_t vertical_speed;        // IEEE float 32
    int32_t latitude;
    int32_t longitude;
    uint32_t altitude;              // IEEE float 32
    uint32_t timestamp;
    uint32_t heading_err_sd;        // IEEE float 32
    uint32_t roll_err_sd;           // IEEE float 32
    uint32_t pitch_err_sd;          // IEEE float 32
    uint32_t north_speed_err_sd;    // IEEE float 32
    uint32_t east_speed_err_sd;     // IEEE float 32
    uint32_t vertical_speed_err_sd; // IEEE float 32
    uint32_t latitude_err_sd;       // IEEE float 32
    uint32_t longitude_err_sd;      // IEEE float 32
    uint32_t altitude_err_sd;       // IEEE float 32
};

// constructor
// on error, should the nav_long_data_t struct be memset to zero?
nav_long::nav_long(char *buf, ssize_t nbytes)
{
    nav_long_pkt *nl;
    uint8_t *phdr;
    uint h, m, s;
    union {
        float f;
        uint32_t u;
    } scratch;
    parse_success = -1; // default to failure

    // Do we have a buffer to parse?
    if (NULL == buf) {
        ROS_ERROR("nav_long: no buffer");
        return;
    }

    // NAVIGATION LONG message must be exactly 90 bytes
    if (nbytes != NAV_LONG_MSG_LENGTH) {
        ROS_ERROR("nav_long: incorrect message length [%ld]", nbytes);
        return;
    }
    
    nl = (nav_long_pkt *)buf;

    // Is packet valid? The first two bytes must be 0x24 0xAA
    phdr = (uint8_t *)&nl->header;
    if (0x24 != *phdr) {
        ROS_ERROR("nav_long: incorrect header byte 0 [%x]", *phdr);
        return;
    }
    phdr++;
    if (0xAA != *phdr) {
        ROS_ERROR("nav_long: incorrect header byte 1 [%x]", *phdr);
        return;
    }
    // Parse packet, shift from network to CPU endian and popuate fields
    nl_data.header = ntohs(nl->header);
    nl_data.user_status = ntohl(nl->user_status);
    nl_data.algo_status[0] = ntohl(nl->algo_status[0]);
    nl_data.algo_status[1] = ntohl(nl->algo_status[1]);

    scratch.u =  ntohl(nl->heading);
    nl_data.heading = scratch.f;

    scratch.u = ntohl(nl->roll);
    nl_data.roll = scratch.f;

    scratch.u = ntohl(nl->pitch);
    nl_data.pitch = scratch.f;

    scratch.u = ntohl(nl->north_speed);
    nl_data.north_speed = scratch.f;

    scratch.u = ntohl(nl->east_speed);
    nl_data.east_speed = scratch.f;

    scratch.u = ntohl(nl->vertical_speed);
    nl_data.vertical_speed = scratch.f;

    nl_data.latitude = ntohl(nl->latitude);
    nl_data.longitude = ntohl(nl->longitude);

    scratch.u = ntohl(nl->altitude);
    nl_data.altitude = scratch.f;

    nl_data.timestamp = ntohl(nl->timestamp);

    scratch.u = ntohl(nl->heading_err_sd);
    nl_data.heading_err_sd = scratch.f;

    scratch.u = ntohl(nl->roll_err_sd);
    nl_data.roll_err_sd = scratch.f;

    scratch.u = ntohl(nl->pitch_err_sd);
    nl_data.pitch_err_sd = scratch.f;

    scratch.u = ntohl(nl->north_speed_err_sd);
    nl_data.north_speed_err_sd = scratch.f;

    scratch.u = ntohl(nl->east_speed_err_sd);
    nl_data.east_speed_err_sd = scratch.f;

    scratch.u = ntohl(nl->vertical_speed_err_sd);
    nl_data.vertical_speed_err_sd = scratch.f;

    scratch.u= ntohl(nl->latitude_err_sd);
    nl_data.latitude_err_sd= scratch.f;

    scratch.u= ntohl(nl->longitude_err_sd);
    nl_data.longitude_err_sd= scratch.f;

    scratch.u = ntohl(nl->altitude_err_sd);
    nl_data.altitude_err_sd = scratch.f;

    /* Sanity check the values. The INS interface library states allowed
     * ranges for most values
     */
    if (0 > nl_data.heading || TWO_PI_RADIANS < nl_data.heading) {
        ROS_ERROR("nav_long. invalid heading [%f]", nl_data.heading);
        return;
    }
    if (-ONE_PI_RADIANS > nl_data.roll || ONE_PI_RADIANS < nl_data.roll) {
        ROS_ERROR("nav_long: invalid roll [%f]", nl_data.roll);
        return;
    }
    if (-ONE_PI_RADIANS/2 > nl_data.pitch || ONE_PI_RADIANS/2 < nl_data.pitch) {
        ROS_ERROR("nav_long: invalid pitch [%f]", nl_data.pitch);
        return;
    }
#if 0 // FIX. Need confirmation from ixBlue regarding data encoding method
    if (-MAX_DEGREES > nl_data.latitude || MAX_DEGREES < nl_data.latitude) {
        ROS_ERROR("nav_long: invalid latitude [%d]",nl_data.latitude );
        return;
    }
    if (-MAX_DEGREES > nl_data.longitude || MAX_DEGREES < nl_data.longitude) {
        ROS_ERROR("nav_long. invalid longitude [%d]", nl_data.longitude);
        return;
    }
#endif
    /* time bits 27-32: hours < 24
     * bits 21-26: minutes    < 60
     * bits 15-20: seconds    < 60
     * bits 0-14: microseconds
     */
    h = (nl_data.timestamp & 0xF8000000) > 27;
    m = (nl_data.timestamp & 0x07E00000) > 21;
    s = (nl_data.timestamp & 0x001F8000) > 15;
    if (HOURS_IN_A_DAY <= h) {
        ROS_ERROR("nav_long. invalid hours [%d]", h);
        return;
    }
    if (MINUTES_IN_AN_HOUR <= m) {
        ROS_ERROR("nav_long. invalid longitude [%d]", m);
        return;
    }
    if (SECONDS_IN_A_MINUTE <= s) { 
       ROS_ERROR("nav_long. invalid longitude [%d]", s);
        return;
    }
    
    // Weeo. We made it.
    parse_success = 0;
}

// destructor
nav_long::~nav_long()
{
    // any clean up required?
}

} /* end namespace c3_protocol */