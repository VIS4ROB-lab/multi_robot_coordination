/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that publishes the odometry transformations for
 *         multiple agents
 * @date   31.07.2019
 */

#include "multi_robot_simulation/odometry_transform_publisher.h"

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "odometry_tranform_publisher_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the publisher
  mrp::OdometryTransformPublisher odometry_transform_publisher(nh, nh_private);
  ros::spin();

  return 0;
}
