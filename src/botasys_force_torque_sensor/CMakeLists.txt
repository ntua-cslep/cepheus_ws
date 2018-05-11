cmake_minimum_required(VERSION 2.8.3)
project(botasys_force_torque_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cmake_modules
  geometry_msgs
)
find_package(Eigen REQUIRED)

catkin_package()

###########
## Build ##
###########

# C++11 Needed
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h)
message(STATUS "Headers are ${HEADERS}")

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node ${HEADERS} src/serial_comms.cpp
                                              src/botasys_force_torque_sensor_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} yaml-cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_calibration_node ${HEADERS} src/serial_comms.cpp
                                              src/botasys_force_torque_sensor_calibration_node.cpp)
target_link_libraries(${PROJECT_NAME}_calibration_node ${catkin_LIBRARIES} yaml-cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_low_pass_filter src/low_pass_filter.cpp)
target_link_libraries(${PROJECT_NAME}_low_pass_filter ${catkin_LIBRARIES})