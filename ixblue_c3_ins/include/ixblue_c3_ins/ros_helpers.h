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

#ifndef IXBLUE_C3_INS_ROS_HELPERS_H
#define IXBLUE_C3_INS_ROS_HELPERS_H

#include "ixblue_c3_ins/c3_protocol.h"


namespace ixblue_c3_ins
{
namespace ros_helpers
{

template<typename MessageT>
MessageT to_ros_message(const c3_protocol::nav_long::nav_long_data_t& data);

}  // namespace ros_helpers
}  // namespace ixblue_c3_ins

#endif  // IXBLUE_C3_INS_ROS_HELPERS_H
