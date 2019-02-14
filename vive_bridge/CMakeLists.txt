cmake_minimum_required(VERSION 3.0.2)
project(vive_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  tf2
  tf2_ros
  dynamic_reconfigure
  rviz_visual_tools
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###########
## Paths ##
###########

## Set OpenVR library path
## ENV reads paths from a system environment variable
set(OPENVR "$ENV{HOME}/lib/openvr")

set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR}/samples/shared)

## OpenVR SDK path
find_library(OPENVR_LIBRARIES
  NAMES
    ## Specify one or more possible names for the library
    openvr_api
  PATHS
    ## Specify directories to search in addition to the default locations
    ${OPENVR}/bin
    ${OPENVR}/lib
  PATH_SUFFIXES
    ## Specify additional subdirectories to check below each directory location otherwise considered
    linux64
  ## If NO_DEFAULT_PATH is specified, then no additional paths are added to the search
  NO_DEFAULT_PATH
  ## If NO_CMAKE_FIND_ROOT_PATH is used then CMAKE_FIND_ROOT_PATH will not be used
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${OPENVR}/headers)

## Set libsurvive library path
## ENV reads paths from a system environment variable
set(SURVIVE "$ENV{HOME}/lib/libsurvive")

## OpenVR SDK path
find_library(SURVIVE_LIBRARIES
  NAMES
    ## Specify one or more possible names for the library
    survive
  PATHS
    ## Specify directories to search in addition to the default locations
    ${SURVIVE}/lib
    ${SURVIVE}/lib/plugins
  ## If NO_DEFAULT_PATH is specified, then no additional paths are added to the search
  NO_DEFAULT_PATH
  ## If NO_CMAKE_FIND_ROOT_PATH is used then CMAKE_FIND_ROOT_PATH will not be used
  NO_CMAKE_FIND_ROOT_PATH
)
set(SURVIVE_INCLUDE_DIR ${SURVIVE}/include/libsurvive
                        ${SURVIVE}/redist
                        ${SURVIVE}/src)

# SET(SURVIVE_SRCS ${SURVIVE_SRCS}
#     ${SURVIVE}/include/libsurvive/poser.h
#     ${SURVIVE}/include/libsurvive/survive.h
#     ${SURVIVE}/include/libsurvive/survive_api.h
#     ${SURVIVE}/include/libsurvive/survive_optimizer.h
#     ${SURVIVE}/include/libsurvive/survive_reproject.h
#     ${SURVIVE}/include/libsurvive/survive_types.h
#     ${SURVIVE}/redist/crc32.c
#     ${SURVIVE}/redist/glutil.c
#     ${SURVIVE}/redist/jsmn.c
#     ${SURVIVE}/redist/jsmntest.c
#     ${SURVIVE}/redist/json_helpers.c
#     ${SURVIVE}/redist/linmath.c
#     ${SURVIVE}/redist/mpfit/mpfit.c
#     ${SURVIVE}/redist/puff.c
#     ${SURVIVE}/redist/symbol_enumerator.c
#     ${SURVIVE}/src/ootx_decoder.c
#     ${SURVIVE}/src/poser.c
#     ${SURVIVE}/src/poser_general_optimizer.c
#     ${SURVIVE}/src/survive.c
#     ${SURVIVE}/src/survive_api.c
#     ${SURVIVE}/src/survive_cal.c
#     ${SURVIVE}/src/survive_config.c
#     ${SURVIVE}/src/survive_default_devices.c
#     ${SURVIVE}/src/survive_disambiguator.c
#     ${SURVIVE}/src/survive_driverman.c
#     ${SURVIVE}/src/survive_imu.c
#     ${SURVIVE}/src/survive_optimizer.c
#     ${SURVIVE}/src/survive_playback.c        
#     ${SURVIVE}/src/survive_plugins.c
#     ${SURVIVE}/src/survive_process.c
#     ${SURVIVE}/src/survive_reproject.c
#     ${SURVIVE}/src/survive_reproject.generated.h
#     ${SURVIVE}/src/survive_sensor_activations.c
#     ${SURVIVE}/src/survive_usb.c
#     ${SURVIVE}/src/survive_kalman.c)
# add_library(survive SHARED ${SURVIVE_SRCS})
# set_target_properties(survive PROPERTIES FOLDER "libraries")

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
  TrackedDevicesStamped.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  GetTrackedDevices.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/DynReconf.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vive_bridge
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENVR_INCLUDE_DIR}
  ${SURVIVE_INCLUDE_DIR}
)
link_directories(${SURVIVE}/lib)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/vive_bridge.cpp
# )
add_library(vive_interface src/vive_interface.cpp)
add_library(survive_interface src/survive_interface.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/vive_node.cpp
  src/vive_interface.cpp
)

add_executable(survive_bridge_node 
  src/vive_node.cpp
  src/survive_interface.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(survive_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OPENVR_LIBRARIES}
  vive_interface
)
target_link_libraries(survive_bridge_node
  ${catkin_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${SURVIVE_LIBRARIES}
  survive_interface
  survive
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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

foreach(dir config launch meshes)
  install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vive_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
