/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that runs the global planning algorithm.
 * @date   29.07.2019
 */

#include "multi_robot_global_planner/multi_robot_global_planner.h"

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "multi_robot_global_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  mrp::MultiRobotGlobalPlanner multi_robot_global_planner_node(nh, nh_private);
  ros::spin();

  return 0;
}
