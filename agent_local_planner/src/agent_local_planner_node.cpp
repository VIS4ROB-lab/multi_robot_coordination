/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that runs the local planning algorithm.
 * @date   02.08.2019
 */

#include "agent_local_planner/agent_local_planner.h"

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "agent_local_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  mrp::AgentLocalPlanner agent_local_planner(nh, nh_private);
  ros::spin();

  return 0;
}
