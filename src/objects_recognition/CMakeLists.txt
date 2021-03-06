cmake_minimum_required(VERSION 2.8.3)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

project(objects_tracker)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
add_message_files(
  FILES
  BoundingBox.msg
  Object.msg
  Objects.msg
)

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
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

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
#  LIBRARIES objects_tracker
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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
)

## Declare a C++ library
# add_library(objects_tracker
#   src/${PROJECT_NAME}/objects_tracker.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(objects_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(objects_tracker_node src/objects_tracker_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(objects_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(objects_tracker_node
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
# install(TARGETS objects_tracker objects_tracker_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_objects_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#################
## My packages ##
#################

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
#find_package(CUDA 5.5 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(HDF5)
find_package(FLANN 1.8)

# OPENMP
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O2")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -O2")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

# PCL
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# # FLANN
# link_directories(${FLANN_LIBRARY_DIRS})
# if(FLANN_CFLAGS)
#  add_definitions(${FLANN_CFLAGS})
# endif(FLANN_CFLAGS) 

# PCL, HDF5 and CATKIN
include_directories( include ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR} ${HDF5_INCLUDE_DIR} ${FLANN_INCLUDE_DIRS})

# point_cloud node
add_executable(point_cloud src/point_cloud.cpp)
target_link_libraries(point_cloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(point_cloud opencv_nonfree249d.lib)

# utilities
add_library(utilities src/utilities/utilities.cpp)
target_link_libraries(utilities ${catkin_LIBRARIES})

# utilities pcl
add_library(pcl src/utilities/pcl.cpp)
target_link_libraries(pcl ${catkin_LIBRARIES} ${PCL_LIBRARIES} utilities)

# utilities ros
add_library(ros src/utilities/ros.cpp)
target_link_libraries(ros ${catkin_LIBRARIES} utilities)

 # utilities bridge
 add_library(bridge src/utilities/bridge.cpp)
 target_link_libraries(bridge ${PCL_LIBRARIES} ${CMAKE_DL_LIBS} ${OpenCV_LIBRARIES})
 
 # utilities opencv
 add_library(opencv src/utilities/opencv.cpp)
 target_link_libraries(opencv ${CMAKE_DL_LIBS} ${OpenCV_LIBRARIES})
 
 # utilities recognition
 add_library(recognition src/recognition/Recognition.cpp)
 target_link_libraries(recognition ${CMAKE_DL_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} opencv bridge pcl)
 add_dependencies(recognition opencv bridge pcl)

# PlaneSegmentation class
add_library(plane_segmentation src/segmentation/PlaneSegmentation.cpp)
target_link_libraries(plane_segmentation ${CMAKE_DL_LIBS} ${PCL_LIBRARIES} pcl)
add_dependencies(plane_segmentation pcl)

# MultiplePlaneSegmentation class
add_library(multiple_plane_segmentation src/segmentation/MultiplePlaneSegmentation.cpp)
target_link_libraries(multiple_plane_segmentation ${CMAKE_DL_LIBS} ${PCL_LIBRARIES} pcl plane_segmentation)
add_dependencies(multiple_plane_segmentation pcl plane_segmentation)

# segmentation node
add_executable(segmentation src/segmentation.cpp)
target_link_libraries(segmentation ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} -lyaml-cpp pcl ros multiple_plane_segmentation)
add_dependencies(segmentation opencv_nonfree249d.lib multiple_plane_segmentation)

# tf_calibration node
add_executable(tf_calibration src/tf_calibration.cpp)
target_link_libraries(tf_calibration ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} -lyaml-cpp)

# tf_publisher node
add_executable(tf_publisher src/tf_publisher.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} -lyaml-cpp)

# objects_to_rviz node
add_executable(objects_to_rviz src/objects_to_rviz.cpp)
target_link_libraries(objects_to_rviz ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENCV_LIB} -lyaml-cpp ros)
 
 # training node
 add_executable(training src/training.cpp)
 target_link_libraries(training ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${PCL_LIBRARIES} ${HDF5_hdf5_LIBRARY} ros utilities pcl recognition)
 add_dependencies(objects_to_rviz ros utilities pcl recognition)
 
 # objects_recognition node
 add_executable(objects_recognition src/objects_recognition.cpp)
 target_link_libraries(objects_recognition ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${PCL_LIBRARIES} ${HDF5_hdf5_LIBRARY} -lyaml-cpp ros utilities pcl recognition)

# # example cuda compiler
# add_executable(segmentation src/segmentation.cpp src/utilities.cu)
# target_link_libraries(segmentation ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} pcl ros ${CUDA_LIBRARIES})
# add_dependencies(segmentation opencv_nonfree249d.lib utilities)