#pragma once

// 今后，headfile.h将是非自定义库的综合

// 系统头文件
#include "ros/ros.h"
#include <ros/package.h>
#include <ros/service_client.h>
#include <iostream>
#include <csignal>
#include <cmath>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <sstream>

// 功能头文件
#include <yaml-cpp/yaml.h>
#include <jsoncpp/json/json.h>

#include <realtime_tools/realtime_publisher.h>

#include <controller_manager_msgs/SwitchController.h>

#include <sensor_msgs/Joy.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"

#include "geometric_shapes/shapes.h"
#include <geometry_msgs/Quaternion.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_messages.h"
#include <geometric_shapes/shape_operations.h>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_listener.h"

#include "std_msgs/String.h"

// moveit！头文件
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/MotionSequenceRequest.h>
#include <moveit_msgs/GetMotionSequence.h>
#include <moveit_msgs/GetMotionPlan.h>

// ompl 头文件
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "manipulator/universal_tools.h"

#include <gripper_modbus/Gripper.h>
#include <balance_com/Balance.h>
#include <locatornew/Location.h>
#include <dashboard/dashboard.h>

#include <std_msgs/Float64.h>
#include <Eigen/Dense>

#include <any>