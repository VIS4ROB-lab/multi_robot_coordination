/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that publishes the trajectory to initialize GPS
 * @date   02.09.2019
 */

#include "multi_robot_simulation/gps_pose_graph_initializer.h"

int main(int argc, char *argv[]) {
  // Get the agent id
  if (argc < 2) {
    ROS_ERROR("[GPS Initializer] Could not initialize, missing agent id!");
    return -1;
  }
  int agent_id = std::strtol(argv[1], nullptr, 0);

  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "gps_pose_graph_initializer_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the publisher
  mrp::GpsPoseGraphInitializer gps_initializer(nh, nh_private, agent_id);
  return 0;
}