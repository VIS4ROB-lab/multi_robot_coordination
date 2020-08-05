/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main class for transforming the pointclouds and sending them to
 *         voxblox in the right format
 * @date   31.07.2019
 */

#include <glog/logging.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace mrp {

class PointcloudTransformator {

public:
  typedef sensor_msgs::PointCloud2 PointcloudROS;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointcloudPCL;

  /**
   * @brief Constructor of the class
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   */
  PointcloudTransformator(const ros::NodeHandle &nh,
                          const ros::NodeHandle &nh_private);

  /**
   * @brief Destructor
   */
  ~PointcloudTransformator() {}

private:
  /**
   * @brief Read the parameters from the server
   * @return True if all parameters were parsed correctly, false otherwise
   */
  bool readParameters();

  /**
   * @brief Method to initialize the subscribers in the right namespace for
   *        all the agents
   */
  void initSubscribers();

  /**
   * @brief Method to initialize the publishers for pointcloud messages
   */
  void initPublisher();

  /**
   * @brief Callback for pointcloud message for different agents
   * @param[in] pcl_msg : pointcloud message
   * @param[in] agent_id : id number for the agent
   */
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg,
                          const uint64_t agent_id);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::vector<ros::Subscriber> pointcloud_subs_;
  std::vector<ros::Publisher> pcl_transform_pubs_;
  std::vector<ros::Publisher> pointcloud_pubs_;

  tf::TransformListener tf_listener_;

  int num_agents_;
  std::string world_frame_;
  std::string odom_frame_;
  std::string agent_frame_;
  bool publish_pcl_world_;

  std::vector<std::string> agents_ns_;

}; // end class PointcloudTransformator

} // end namespace mrp
