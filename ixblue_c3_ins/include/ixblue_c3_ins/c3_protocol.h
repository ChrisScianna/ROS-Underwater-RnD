/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2021, QinetiQ, Inc.
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
