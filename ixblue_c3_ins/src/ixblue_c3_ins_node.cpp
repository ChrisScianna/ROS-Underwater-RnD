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

/* Overview:
 * This node has provides navigation information to the system. It has
 * two functions. First, it "consumes" UDP data from the ixBlue C3 inertial
 * navigation system (INS). Once the UDP packet data is decoded, the information
 * is published to the other ROS nodes for their handling.
 */

#include <ros/ros.h>

#include "ixblue_c3_ins/ixblue_c3_ins.h"

#define NODE_NAME "ixblue_c3_ins"
#define NODE_VERSION "0.1.0"

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  ROS_INFO("Node version: [%s]", NODE_VERSION);

  try
  {
    ixblue_c3_ins::ixBlueC3InsDriver driver;

    driver.spin();
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR("%s", e.what());
    return -1;
  }
  return 0;
}

