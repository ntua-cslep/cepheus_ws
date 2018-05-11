cmake_minimum_required(VERSION 2.8.3)
project(phase_space)

set(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
	roscpp 
	visualization_msgs 
	std_msgs 
	geometry_msgs 
	tf 
	message_generation
	cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
 	FILES
	PhaseSpaceMarker.msg
	PhaseSpaceMarkerArray.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   visualization_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phase_space
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

# for the precompiled libraries
link_directories(lib)

add_library(pose_tracking_filter
	src/PoseTrackingFilter.cpp
)

## Declare a cpp executable
add_executable(phase_space_node 
	src/phase_space_node.cpp
)

add_executable(phase_space_rviz_publisher 
	src/phase_space_rviz_publisher.cpp
)

add_executable(track_body_node 
	src/track_body_node.cpp
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(phase_space_node
	phase_space_generate_messages_cpp
)
add_dependencies(phase_space_rviz_publisher
	phase_space_generate_messages_cpp
)
add_dependencies(track_body_node
	phase_space_generate_messages_cpp
	pose_tracking_filter
)

## Specify libraries to link a library or executable target against
target_link_libraries(phase_space_node
	owlsock
	${catkin_LIBRARIES} 
)
target_link_libraries(phase_space_rviz_publisher
	${catkin_LIBRARIES} 
)
target_link_libraries(track_body_node
	pose_tracking_filter
	${catkin_LIBRARIES} 
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS phase_space phase_space_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_phase_space.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
