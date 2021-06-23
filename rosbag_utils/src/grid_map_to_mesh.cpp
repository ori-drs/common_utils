/**
 * @file grid_map_to_mesh.hpp
 * @brief A converter of grid_map messages to a mesh.
 * @author Oliwier Melon (omelon@robots.ox.ac.uk), Yiduo Wang
 * (ywang@robots.ox.ac.uk)
 * @bug No known bugs.
 * @date 17/05/2021
 */

#include "grid_map_to_mesh.hpp"

#include <pcl-1.10/pcl/io/obj_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>

GridMapToMesh::GridMapToMesh(ros::NodeHandle& nh) : ros_nh_(nh) {
  readParameters();
  ros_sub_grid_map_ = ros_nh_.subscribe(
      topic_grid_map_, 1, &GridMapToMesh::callbackGridMapMsg, this);
  ros_pub_pointcloud_ =
      ros_nh_.advertise<sensor_msgs::PointCloud2>(topic_pointcloud_, 1, false);
  ros_pub_grid_map_out_ =
      ros_nh_.advertise<grid_map_msgs::GridMap>(topic_grid_map_out_, 1, false);

  // TF
  tf_listener_ =
      std::make_shared<tf::TransformListener>();  // TODO: Upgrade to
                                                  // tf2_ros::TransformListener

  pointcloud_.clear();
}

GridMapToMesh::~GridMapToMesh() {
  if (pointcloud_.size()) saveFullPointCloudToMesh();
}

bool GridMapToMesh::readParameters() {
  if (!ros_nh_.getParam("/grid_map_to_mesh/topic_grid_map", topic_grid_map_)) {
    ROS_ERROR("Could not read parameter `/grid_map_to_mesh/topic_grid_map`.");
    return false;
  }

  if (!ros_nh_.getParam("/grid_map_to_mesh/topic_pointcloud",
                        topic_pointcloud_)) {
    ROS_ERROR("Could not read parameter `/grid_map_to_mesh/topic_pointcloud`.");
    return false;
  }

  if (!ros_nh_.getParam("/grid_map_to_mesh/topic_grid_map_out",
                        topic_grid_map_out_)) {
    ROS_ERROR("Could not read parameter `/grid_map_to_mesh/topic_grid_map_out`.");
    return false;
  }

  std::string path_to_save_mesh;
  if (!ros_nh_.getParam("/grid_map_to_mesh/path_to_save_mesh",
                        path_to_save_mesh)) {
    ROS_ERROR(
        "Could not read parameter `/grid_map_to_mesh/path_to_save_mesh`.");
    return false;
  }
  path_to_save_mesh_ = path_to_save_mesh + ".obj";
  return true;
}

void GridMapToMesh::callbackGridMapMsg(const grid_map_msgs::GridMap& msg) {
  grid_map::GridMapRosConverter::fromMessage(msg, grid_map_);
  ros::Time tic = ros::Time::now();
  getTF();
  convertGridMapToPointCloud();
  publishPointcloud();
  ros::Time toc = ros::Time::now();
  ros::Duration time = toc - tic;
  std::cout << "convertGridMapToPointCloud took " << time.toSec()
            << " seconds.\n";
}

void GridMapToMesh::convertGridMapToPointCloud() {
  pointcloud_local_.clear();
  bool success = true;
  double h = 0.;
  grid_map::Position robot_xy = grid_map_.getPosition();

  robot_xy_history_.push_back(robot_xy);
  if (robot_xy_history_.size() > 30) {
    robot_xy_history_.pop_front();
  }

  grid_map::GridMap grid_map_submap;
  grid_map_submap = grid_map_.getSubmap(grid_map_.getPosition(),
                                        grid_map::Length(2., 2.), success);
  grid_map_msgs::GridMap output_grid_map_msg;
  grid_map::GridMapRosConverter::toMessage(grid_map_submap, output_grid_map_msg); 
  ros_pub_grid_map_out_.publish(output_grid_map_msg);

  for (grid_map::GridMapIterator iterator(grid_map_submap);
       !iterator.isPastEnd(); ++iterator) {
    bool add_point = true;
    const grid_map::Index index(*iterator);
    grid_map::Position p;
    grid_map_submap.getPosition(index, p);
    h = grid_map_submap.atPosition("elevation", p);
    if (std::isnan(h) || std::isinf(h)) continue;
    // if (std::hypot(p.x() - robot_xy.x(), p.y() - robot_xy.y()) < 0.5)
    // continue;
    for (int i = 0; i < robot_xy_history_.size(); i++) {
      if (std::hypot(p.x() - robot_xy_history_[i].x(),
                     p.y() - robot_xy_history_[i].y()) < (0.3)) {
        add_point = false;
        break;
      }
    }
    if (!add_point) continue;
    if (h > m_T_boi_.translation().z() + 0.25) continue;

    pcl::PointXYZ point(p.x(), p.y(), h);
    pointcloud_local_.push_back(point);
  }
  filterPointcloud(pointcloud_local_, pointcloud_filtered_);
  pointcloud_local_ = pointcloud_filtered_;
  removeStatisticalOutlier(pointcloud_local_, pointcloud_filtered_);

  pointcloud_ += pointcloud_filtered_;
  pcl::PointCloud<pcl::PointXYZ> full_pointcloud_filtered;
  filterPointcloud(pointcloud_, full_pointcloud_filtered);
  pointcloud_ = full_pointcloud_filtered;

  std::cout << "pointcloud_.size() = " << pointcloud_.size() << "\n";
}

void GridMapToMesh::publishPointcloud() {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(pointcloud_filtered_, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  ros_pub_pointcloud_.publish(msg);
}

void GridMapToMesh::publishGridMap() {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(grid_map_, msg);

}

void GridMapToMesh::filterPointcloud(
    pcl::PointCloud<pcl::PointXYZ>& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_ptr = cloud_in.makeShared();
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud_in_ptr);
  voxel_filter.setLeafSize(grid_map_.getResolution(), grid_map_.getResolution(),
                           4. * grid_map_.getResolution());
  voxel_filter.filter(cloud_out);
}

void GridMapToMesh::removeStatisticalOutlier(
    pcl::PointCloud<pcl::PointXYZ>& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out) {
  std::cout << "[GridMapToMesh::removeStatisticalOutlier] cloud_in.size() = "
            << cloud_in.size() << "\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_ptr = cloud_in.makeShared();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_outlier_removal;
  stat_outlier_removal.setInputCloud(cloud_in_ptr);
  stat_outlier_removal.setMeanK(100);
  stat_outlier_removal.setStddevMulThresh(1.5);
  stat_outlier_removal.filter(cloud_out);
}
void GridMapToMesh::getTF() {
  tf::StampedTransform m_T_boi_transform;
  tf_listener_->waitForTransform(
      "map", "body", ros::Time(0),
      ros::Duration(2.0));  // TODO: hard-coded frame names
  try {
    tf_listener_->lookupTransform(
        "map", "body", ros::Time(0),
        m_T_boi_transform);  // TODO: hard-coded frame names
    tf::transformTFToEigen(m_T_boi_transform, m_T_boi_);
  } catch (const tf::TransformException& e) {
    ROS_ERROR_STREAM("Couldn't find transform from frame ["
                     << "body odom icp"
                     << "] to frame ["
                     << "map"
                     << "]. " << e.what());
  }
}

void GridMapToMesh::saveFullPointCloudToMesh() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr = pointcloud_.makeShared();

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_ptr);
  n.setInputCloud(pointcloud_ptr);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(pointcloud_, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(3. * grid_map_.getResolution());

  // Set typical values for the parameters
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
  gp3.setMinimumAngle(M_PI / 18);        // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  // Save as .obj
  pcl::io::savePCDFileASCII(path_to_save_mesh_ + ".pcd", *cloud_with_normals);
  pcl::io::savePLYFileBinary(path_to_save_mesh_ + ".ply", *cloud_with_normals);
  pcl::io::saveOBJFile(path_to_save_mesh_, triangles);
}