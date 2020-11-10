
#include "pose_estimator/pose_estimator.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_estimator_node");
  ros::NodeHandle n;
  ROS_INFO("Starting Pose Estimator node Version: [%s]", NODE_VERSION);
  n.setParam("/version_numbers/pose_estimator", NODE_VERSION);
  qna::robot::PoseEstimatorNode poseObject(n);
  poseObject.spin();
  return 0;
}