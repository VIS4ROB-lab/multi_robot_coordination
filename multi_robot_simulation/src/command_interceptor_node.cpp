/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that reads a command in the odometry frame and
 *         transforms it into the controller reference frame
 * @date   03.09.2019
 */

#include "multi_robot_simulation/command_interceptor.h"

int main(int argc, char *argv[]) {
  // Get the agent id
  if (argc < 2) {
    ROS_ERROR("[Command Interceptor] Could not initialize, missing agent id!");
    return -1;
  }
  int agent_id = std::strtol(argv[1], nullptr, 0);

  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "command_interceptor_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the initializer
  mrp::CommandInterceptor command_interceptor(nh, nh_private, agent_id);
  ros::spin();
  return 0;
}