cmake_minimum_required(VERSION 2.8.3)
project(eigen_utils)

add_compile_options(-std=c++11)

# this is just to make the "catkin_package" macro available
find_package(catkin)

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include
               LIBRARIES eigen_utils
               DEPENDS EIGEN3)

add_library(${PROJECT_NAME} SHARED src/eigen_numerical.cpp
                                   src/eigen_rand.cpp
                                   src/eigen_rigidbody.cpp
                                   src/eigen_utils_common.cpp)

install(TARGETS eigen_utils
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PACKAGE_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

