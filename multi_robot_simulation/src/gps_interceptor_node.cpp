/*
 * Copyright (c) 2020, Vision for Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Vision for Robotics Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
