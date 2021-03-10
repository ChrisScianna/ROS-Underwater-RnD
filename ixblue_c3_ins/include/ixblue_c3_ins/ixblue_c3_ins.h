/*  QinetiQ North America (QNA)
 *  350 Second Avenue
 *  Waltham, MA 02451
 *
 *  Proprietary Licensed Information.
 *  Not to be duplicated, used, or disclosed,
 *  except under terms of the license.
 *
 *  Copyright © 2021 QinetiQ North America  All rights reserved.
 */

#ifndef IXBLUE_C3_INS_IXBLUE_C3_INS_H
#define IXBLUE_C3_INS_IXBLUE_C3_INS_H

#include <ros/ros.h>

#include <auv_interfaces/StateStamped.h>
#include <diagnostic_tools/diagnosed_publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "ixblue_c3_ins/c3_protocol.h"


namespace ixblue_c3_ins
{

class ixBlueC3InsDriver
{
public:
  ixBlueC3InsDriver();
  ~ixBlueC3InsDriver();

  void spin();

private:
  void publish(const c3_protocol::nav_long::nav_long_data_t& data);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher nav_long_pub_;

  qna::diagnostic_tools::DiagnosedPublisher<
    auv_interfaces::StateStamped> state_pub_;
  diagnostic_updater::Updater diagnostics_updater_;

  int fd_;  // Listening UDP socket to "consume" data from the INS
};

}  // namespace ixblue_c3_ins

#endif  // IXBLUE_C3_INS_IXBLUE_C3_INS_H
