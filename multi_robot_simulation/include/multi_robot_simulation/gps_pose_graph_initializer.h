/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main class for sending the commands to the controller to perform
 *         initial movement to initialize the GPS for the pose-graph
 * @date   02.09.2019
 */

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <glog/logging.h>

namespace mrp {

class GpsPoseGraphInitializer {
public:
    /**
     * @brief Constructor
     * @param[in] nh : ROS node handle
     * @param[in] nh_private : Private ROS node handle
     * @param[in] agent_id : ID of the agent
     */
    GpsPoseGraphInitializer(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private,
                    const int agent_id);

    /**
     * @brief Destructor
     */
    virtual ~GpsPoseGraphInitializer();

private:
    /**
     * @brief Callback to check if we have odometry information (ground truth
     *        odometry)
     * @param[in] odom_msg : odometry message
     */
    void odometryCallback(const nav_msgs::OdometryConstPtr &odom_msg);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber odom_sub_;
    ros::Publisher trajectory_cmd_pub_;
    ros::Publisher command_vis_pub_;

    mav_msgs::EigenOdometry odometry_;
    bool has_odometry_;
    int agent_id_;
    std::string frame_id_;
};  // end class vins initializer

} // end namespace mrp