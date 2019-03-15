cmake_minimum_required(VERSION 2.8.3)
project(concise_args)
find_package(catkin REQUIRED)

catkin_package(INCLUDE_DIRS include)

include_directories(include)

add_executable(${PROJECT_NAME}_test test/concise-args-test.cpp)
add_executable(${PROJECT_NAME}_simple_test test/concise-args-simple-test.cpp)

install(FILES ${INCLUDE_OUTPUT_PATH}/ConciseArgs DESTINATION  ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

