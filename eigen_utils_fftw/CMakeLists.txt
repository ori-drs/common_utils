cmake_minimum_required(VERSION 2.8.3)
project(eigen_utils_fftw)

find_package(catkin REQUIRED COMPONENTS eigen_utils)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PkgConfig)

# double precision library
pkg_check_modules(FFTW3 fftw3)
include_directories(${FFTW3_INCLUDE_DIRS})

# single precision library
pkg_check_modules(FFTW3F fftw3f)
include_directories(${FFTW3F_INCLUDE_DIRS})


if(NOT FFTW3_FOUND)
    message(WARNING "fftw not found. Not building ${PROJECT_NAME} \n\
You can install it by running 'sudo apt-get install libfftw3-dev'") 
    return()             
endif()

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS eigen_utils
               DEPENDS EIGEN3 FFTW3 FFTW3F)

include_directories(include)
add_library(${PROJECT_NAME} SHARED src/eigen_fftw.cpp)
target_link_libraries(${PROJECT_NAME} ${FFTW3_LIBRARIES})


#build the test executable
add_executable(eigen-test-fftw test/test_eigen_fftw.cpp)
target_link_libraries(eigen-test-fftw ${PROJECT_NAME} ${FFTW3_LIBRARIES} ${FFTW3F_LIBRARIES})

