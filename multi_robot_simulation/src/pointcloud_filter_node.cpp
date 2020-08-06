/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that removes points within a box from a pointcloud
 * @date   31.07.2019
 */

#include "multi_robot_simulation/pointcloud_filter.h"

int main(int argc, char** argv) {
  // Get the agent id
  if (argc < 2) {
    ROS_ERROR("[PCL Filter] Could not initialize filter, missing agent id!");
    return -1;
  }
  int agent_id = std::strtol(argv[1], nullptr, 0);

  // Initialize ROS, start node.
  ros::init(argc, argv, "pointcloud_filter_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  pointcloud_filter::PointcloudFilter pcl_filter_node(nh, nh_private, agent_id);

  ros::spin();
  return 0;
}
