set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
cmake_minimum_required(VERSION 3.0.2)
project(manipulator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ompl
  controller_manager_msgs
  geometry_msgs
  joy
  message_generation
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  realtime_tools
  roscpp
  rospy
  serial
  std_msgs
  tf2
  gripper_modbus
  locatornew
  balance_com
  dashboard
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(gripper_modbus REQUIRED)
find_package(dashboard REQUIRED)
find_package(balance_com REQUIRED)

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
#   controller_manager_msgs#   geometry_msgs#   std_msgs
# )
generate_messages(
  DEPENDENCIES
  std_msgs
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
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
 LIBRARIES manipulator
 CATKIN_DEPENDS controller_manager_msgs geometry_msgs joy moveit_core moveit_ros_planning_interface realtime_tools roscpp rospy serial std_msgs tf2 moveit_visual_tools message_runtime
 DEPENDS OMPL
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
  ${OMPL_INCLUDE_DIRS}
)

# Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/manipulator.cpp
# )
set(SELF_ROBOTCPP
  src/robot_arm.cpp
  src/robot_capsulation.cpp
  src/station_func.cpp
)
set(SELF_TOOLCPP
  src/universal_tools.cpp
)

set(SELF_LIBRARIES
  yaml-cpp
  jsoncpp
)

# add_library(universal_tools_lib
#   src/universal_tools.cpp
# )
# add_library(robot_arm_lib
#   src/robot_arm.cpp
#   src/robot_capsulation.cpp
#   src/station_func.cpp
# )

# target_link_libraries(robot_arm_lib 
#   universal_tools_lib)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/manipulator_node.cpp)
add_executable(A_manipulator src/A_manipulator.cpp
  ${SELF_ROBOTCPP}
  ${SELF_TOOLCPP})

add_executable(B_record src/B_record.cpp
  ${SELF_ROBOTCPP}
  ${SELF_TOOLCPP})

# add_executable(C_topic_pubsub src/C_topic_pubsub.cpp
#   ${SELF_ROBOTCPP}
  # ${SELF_TOOLCPP})

# add_executable(D_pubmeasure src/D_pubmeasure.cpp
#   ${SELF_TOOLCPP})

# add_executable(Y_testfield src/Y_testfield.cpp   
#   ${SELF_ROBOTCPP}
#   ${SELF_TOOLCPP})
# add_executable(Z_testfield src/Z_testfield.cpp
#   ${SELF_ROBOTCPP}
#   ${SELF_TOOLCPP})

# add_executable(JR_testfield src/JR_testfield.cpp
#   ${SELF_ROBOTCPP}
#   ${SELF_TOOLCPP})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# 官方库
set(OFFICIAL_LIBRARIES
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

target_link_libraries(A_manipulator
  ${OFFICIAL_LIBRARIES}
  ${SELF_LIBRARIES}
)

target_link_libraries(B_record
  ${OFFICIAL_LIBRARIES}
  ${SELF_LIBRARIES}
)

# target_link_libraries(C_topic_pubsub
#   ${OFFICIAL_LIBRARIES}
#   ${SELF_LIBRARIES}
# )

# target_link_libraries(D_pubmeasure
#   ${OFFICIAL_LIBRARIES}
#   ${SELF_LIBRARIES}
# )

# target_link_libraries(Y_testfield
#   ${OFFICIAL_LIBRARIES}
#   ${SELF_LIBRARIES}
# )
# target_link_libraries(Z_testfield
#   ${OFFICIAL_LIBRARIES}
#   ${SELF_LIBRARIES}
# )

# target_link_libraries(JR_testfield
#   ${OFFICIAL_LIBRARIES}
#   ${SELF_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/fake_command.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_manipulator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
