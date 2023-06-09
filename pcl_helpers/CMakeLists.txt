cmake_minimum_required(VERSION 2.8.3)
project(pcl_helpers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(PCL 1.5 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

generate_dynamic_reconfigure_options(
  cfg/voxel_filter_node.cfg
  cfg/passthrough_filter_node.cfg
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
#  LIBRARIES matrix_transform
#  CATKIN_DEPENDS pcl roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)


link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

## Declare a C++ library
# add_library(matrix_transform
#   src/${PROJECT_NAME}/matrix_transform.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(matrix_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(matrix_transform_node src/matrix_transform_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(matrix_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(matrix_transform_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS matrix_transform matrix_transform_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_matrix_transform.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(real_time_transform src/real_time_transform.cpp)
#target_link_libraries(real_time_transform ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(statistical_removal src/statistical_removal.cpp)
#target_link_libraries(statistical_removal ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${catkin_LIBRARIES})

#add_executable (correspondence_grouping src/correspondence_grouping.cpp)
#target_link_libraries (correspondence_grouping ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES}) #${PCL_LIBRARIES}

#add_executable (random_sample_consensus src/random_sample_consensus.cpp)
#target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})

add_executable(real_time_transform src/real_time_transform.cpp)
target_link_libraries(real_time_transform ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${catkin_LIBRARIES})

#add_executable (real_time_computed_points src/real_time_computed_points.cpp)
#target_link_libraries (real_time_computed_points ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(real_time_computed_points ${PROJECT_NAME}_gencfg)

add_executable(voxel_filter_node src/voxel_filter_node.cpp)
add_executable(passthrough_filter_node src/passthrough_filter_node.cpp)
add_executable(statistical_removal src/statistical_removal.cpp)
add_executable(edges_detec src/edges_detec.cpp)
add_executable(cloud_difference src/cloud_difference.cpp)
add_executable(plane_model_segmentation src/plane_model_segmentation.cpp)
add_executable(plane_segmentation_pcd src/plane_segmentation_pcd.cpp)
add_executable(project_inliner src/project_inliner.cpp)

target_link_libraries(voxel_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(passthrough_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(statistical_removal ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(edges_detec ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(cloud_difference ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(plane_model_segmentation ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(plane_segmentation_pcd ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(project_inliner ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_dependencies(voxel_filter_node ${PROJECT_NAME}_gencfg)
add_dependencies(passthrough_filter_node ${PROJECT_NAME}_gencfg)
add_dependencies(statistical_removal ${PROJECT_NAME}_gencfg)
add_dependencies(edges_detec ${PROJECT_NAME}_gencfg)
add_dependencies(cloud_difference ${PROJECT_NAME}_gencfg)
add_dependencies(plane_model_segmentation ${PROJECT_NAME}_gencfg)
add_dependencies(plane_segmentation_pcd ${PROJECT_NAME}_gencfg)
add_dependencies(project_inliner ${PROJECT_NAME}_gencfg)
