cmake_minimum_required(VERSION 2.8.3)
project(tf_remapper)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    rospy
    tf2_geometry_msgs
    tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    tf2_geometry_msgs
    tf2_ros
)

install(PROGRAMS bin/${PROJECT_NAME}/${PROJECT_NAME}.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
