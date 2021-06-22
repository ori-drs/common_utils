#include "grid_map_to_mesh.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grid_map_to_mesh");
  ros::NodeHandle node_handle;
  GridMapToMesh grid_map_to_mesh(node_handle);
  ros::spin();
  return 0;
}
