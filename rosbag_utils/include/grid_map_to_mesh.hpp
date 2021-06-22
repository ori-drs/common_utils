/**
 * @file grid_map_to_mesh.hpp
 * @brief A converter of grid_map messages to a mesh.
 * @author Oliwier Melon (omelon@robots.ox.ac.uk), Yiduo Wang
 * (ywang@robots.ox.ac.uk)
 * @bug No known bugs.
 * @date 17/05/2021
 */

#pragma once

#include <pcl-1.10/pcl/common/common.h>
#include <pcl-1.10/pcl/point_types.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <grid_map_ros/grid_map_ros.hpp>

class GridMapToMesh {
 public:
  /**
   * @brief Construct the ROS-based converter
   * @param node_handle
   */
  GridMapToMesh(ros::NodeHandle& nh);
  ~GridMapToMesh();

 private:
  bool readParameters();
  /**
   * @brief The callback function used to get the grid_map message
   * @param[in] msg grid_map message
   */
  void callbackGridMapMsg(const grid_map_msgs::GridMap& msg);
  void convertGridMapToPointCloud();
  void saveFullPointCloudToMesh();
  void publishPointcloud();
  void publishGridMap();
  void filterPointcloud(pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                        pcl::PointCloud<pcl::PointXYZ>& cloud_out);
  void removeStatisticalOutlier(pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                                pcl::PointCloud<pcl::PointXYZ>& cloud_out);

  void getTF();
  grid_map::GridMap grid_map_;        ///< local grid_map
  grid_map::GridMap grid_map_submap;  ///< sub area of the local grid_map
  pcl::PointCloud<pcl::PointXYZ> pointcloud_;        ///< accumulated pointcloud
  pcl::PointCloud<pcl::PointXYZ> pointcloud_local_;  ///< accumulated pointcloud
  pcl::PointCloud<pcl::PointXYZ>
      pointcloud_filtered_;           ///< accumulated pointcloud
  ros::NodeHandle ros_nh_;            ///< node handle
  ros::Subscriber ros_sub_grid_map_;  ///< subscriber grid_map_msg

  ros::Publisher ros_pub_grid_map_out_;  ///< publisher grid_map_msg
  ros::Publisher ros_pub_pointcloud_;    ///< publisher grid_map_msg
  std::string topic_grid_map_;           ///< topic grid_map
  std::string topic_grid_map_out_;       ///< topic grid_map
  std::string topic_pointcloud_;         ///< topic pointcloud
  std::string
      path_to_save_mesh_;  ///< path to save mesh (relative to /home/user/)

  // TF
  std::shared_ptr<tf::TransformListener> tf_listener_;
  Eigen::Isometry3d m_T_boi_;
  std::deque<grid_map::Position> robot_xy_history_;
};