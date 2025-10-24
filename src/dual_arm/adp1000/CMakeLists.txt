cmake_minimum_required(VERSION 3.0.2)
project(adp1000)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## 关键修改1：添加 message_generation（用于生成服务代码）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation  # 服务/消息生成依赖，必须添加
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

## 关键修改2：声明服务文件（对应我们创建的 DripperSet.srv）
## 服务文件需放在包的 srv/ 目录下，命名为 DripperSet.srv
# add_service_files(
#   FILES
# )

## 关键修改3：生成服务代码（依赖 std_msgs）
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # 服务文件中使用的消息类型依赖
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## 无需修改（本功能包不涉及动态参数配置）
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## 关键修改4：添加 message_runtime（服务运行时依赖）
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES adp1000
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime  # 补充 message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## 无需修改（本功能包为Python节点，无C++库/可执行文件）
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/adp1000.cpp
# )

#############
## Install ##
#############

## 关键修改5：确认Python脚本安装路径（需与实际脚本存放目录一致）
## 假设服务化代码放在包的 scripts/ 目录下，文件名为 adp1000_dripper_node.py
catkin_install_python(PROGRAMS
  scripts/adp1000.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 其他安装配置无需修改（无C++可执行文件、头文件等）
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

## 无需修改（可选添加测试，非必需）
# catkin_add_gtest(${PROJECT_NAME}-test test/test_adp1000.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()