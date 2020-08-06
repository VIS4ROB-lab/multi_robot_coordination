/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that publishes the trajectory to initialize VINS-Mono
 * @date   31.08.2019
 */

#include "multi_robot_simulation/vins_initializer.h"

int main(int argc, char *argv[]) {
  // Get the agent id
  if (argc < 2) {
    ROS_ERROR("[Vins Initializer] Could not initialize, missing agent id!");
    return -1;
  }
  int agent_id = std::strtol(argv[1], nullptr, 0);

  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "vins_initializer_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the initializer
  mrp::VinsInitializer vins_initializer(nh, nh_private, agent_id);
  return 0;
}