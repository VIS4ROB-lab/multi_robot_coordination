/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that publishes the pointclouds to be used by voxblox
 * @date   31.07.2019
 */

#include "multi_robot_simulation/pointcloud_transformator.h"

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "pointcloud_transformator_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the publisher
  mrp::PointcloudTransformator pointcloud_transformator(nh, nh_private);
  ros::spin();

  return 0;
}