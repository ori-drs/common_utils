cmake_minimum_required(VERSION 2.8.3)
project(eigen_utils)

add_compile_options(-std=c++14)

# this is just to make the "catkin_package" macro available
find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include
               LIBRARIES eigen_utils
               DEPENDS EIGEN3)

add_library(${PROJECT_NAME} SHARED src/eigen_numerical.cpp
                                   src/eigen_rand.cpp
                                   src/eigen_rigidbody.cpp
                                   src/eigen_utils_common.cpp)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(quat_rpy_tests test/test_transforms.cpp)
  target_link_libraries(quat_rpy_tests ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

