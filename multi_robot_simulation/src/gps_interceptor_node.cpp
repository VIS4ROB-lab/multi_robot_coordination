/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that read and republish a gps message
 * @date   03.09.2019
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

// Create the variables necessary for the node
ros::Subscriber gps_sub;
ros::Publisher gps_pub;

void gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg) {
  // Overwrite the stamp and then republish as it it
  sensor_msgs::NavSatFix gps_mirrored(*gps_msg);
  gps_mirrored.header.stamp = ros::Time::now();
  gps_pub.publish(gps_mirrored);
}

int main(int argc, char *argv[]) {

  // Get the agent id
  if (argc < 2) {
    ROS_ERROR("[GPS Interceptor] Could not initialize, missing agent id!");
    return -1;
  }
  int agent_id = std::strtol(argv[1], nullptr, 0);

  ros::init(argc, argv, "gps_interceptor_node");
  ros::NodeHandle nh;

  // Initialize publisher and subscriber
  gps_pub = nh.advertise<sensor_msgs::NavSatFix>(
          "fix_" + std::to_string(agent_id), 10);
  gps_sub = nh.subscribe("gps", 10, gpsCallback);

  // Spin
  ros::spin();
  return 0;
}