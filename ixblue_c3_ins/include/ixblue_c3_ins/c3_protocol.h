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

#ifndef IXBLUE_C3_INS_C3_PROTOCOL_H
#define IXBLUE_C3_INS_C3_PROTOCOL_H

#include <stdint.h>
#include <sys/types.h>

namespace ixblue_c3_ins
{
namespace c3_protocol
{

/* contents of a ixBlue C3 'navigation long' packetconverted
 * into a host data structure for publishing to other nodes.
 */
class nav_long
{
public:
  // constructor/destructor
  nav_long(char *buf, size_t nbytes);
  ~nav_long();
  // accessors

  struct nav_long_data_t
  {
    /* Contents of a ixBlue c3 'navigation long' packet
       See pages 370 and 371 (numbered) of the INS- Interface Library
    */
    uint16_t header;
    uint32_t user_status;
    uint32_t algo_status[2];
    float heading;                // IEEE float 32
    float roll;                   // IEEE float 32
    float pitch;                  // IEEE float 32
    float north_speed;            // IEEE float 32
    float east_speed;             // IEEE float 32
    float vertical_speed;         // IEEE float 32
    int32_t latitude;
    int32_t longitude;
    float altitude;               // IEEE float 32
    uint32_t timestamp;
    float heading_err_sd;         // IEEE float 32
    float roll_err_sd;            // IEEE float 32
    float pitch_err_sd;           // IEEE float 32
    float north_speed_err_sd;     // IEEE float 32
    float east_speed_err_sd;      // IEEE float 32
    float vertical_speed_err_sd;  // IEEE float 32
    float latitude_err_sd;        // IEEE float 32
    float longitude_err_sd;       // IEEE float 32
    float altitude_err_sd;        // IEEE float 32

    static constexpr float latlon_resolution = 8.382E-8f;
  };

  const nav_long_data_t& get_data() const { return nl_data; }

  int get_parse_success() const { return parse_success; }

private:
  int parse_success;  // flag for signaling parsing failure

  struct nav_long_data_t nl_data;
};

}  // namespace c3_protocol
}  // namespace ixblue_c3_ins


#endif  // IXBLUE_C3_INS_C3_PROTOCOL_H
