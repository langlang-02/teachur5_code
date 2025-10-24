// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-07-02
 *
 */
//----------------------------------------------------------------------

#include <twist_controller/twist_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include "twist_controller/TwistControllerConfig.h"

namespace ros_controllers_cartesian
{
bool TwistController::init(TwistCommandInterface* hw, ros::NodeHandle& n)
{
  std::string frame_id;
  std::string base_frame_id = "base";
  if (!n.getParam("frame_id", frame_id))
  {
    ROS_ERROR_STREAM("Required parameter " << n.resolveName("frame_id") << " not given");
    return false;
  }
  if (!n.getParam("base_frame_id", base_frame_id))
  {
    ROS_WARN_STREAM("Use default parameter: base_frame_id = base");
  }
  std::cout<<"-----"<<frame_id<<"--------"<<std::endl;
  std::cout<<"-----"<<base_frame_id<<"--------"<<std::endl;
  
  gain_ = n.param("twist_gain", 0.1);

  server_.reset(new dynamic_reconfigure::Server<twist_controller::TwistControllerConfig>(n));
  server_->setCallback(boost::bind(&TwistController::reconfigureCallback, this, _1, _2));

  handle_ = hw->getHandle(frame_id);
  realtime_pub_.reset(new realtime_tools::RealtimePublisher
    <cartesian_state_msgs::CartesianState>(n, "ee_state", 4));
  realtime_pub_->msg_.header.frame_id = base_frame_id;
  realtime_pub_->msg_.child_frame_id = base_frame_id;
  twist_sub_ = n.subscribe<geometry_msgs::Twist>("command", 1, &TwistController::twistCallback, this,
                                                 ros::TransportHints().reliable().tcpNoDelay());

  std::vector<std::string> joint_names;
  if (!n.getParam("joints", joint_names))
  {
    ROS_ERROR_STREAM("Failed to read required parameter '" << n.resolveName("joints") << ".");
    return false;
  }

  for (auto& name : joint_names)
  {
    hw->claim(name);
  }

  return true;
}

void TwistController::starting(const ros::Time& time)
{
  geometry_msgs::Twist twist;
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
  command_buffer_.writeFromNonRT(twist);
}

void TwistController::update(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Pose pose_current = handle_.getPose();
  geometry_msgs::Twist twist_current = handle_.getTwist();
  geometry_msgs::Accel accel_current = handle_.getAccel();
  // TODO:Implement your control law here
  // auto twist_desired = *command_buffer_.readFromRT(); 
  // TODO:利用TF转为Eigen，使用PD控制律
  // auto err = twist_current - twist_desired;

  handle_.setCommand(*command_buffer_.readFromRT());
  // try to publish
  if (realtime_pub_->trylock())
  {
    realtime_pub_->msg_.header.stamp = time;
    realtime_pub_->msg_.pose = pose_current;
    realtime_pub_->msg_.twist = twist_current;
    realtime_pub_->msg_.accel = accel_current;
    realtime_pub_->unlockAndPublish();
  }
}

void TwistController::twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
  geometry_msgs::Twist twist;
  twist.linear.x = gain_ * msg->linear.x;
  twist.linear.y = gain_ * msg->linear.y;
  twist.linear.z = gain_ * msg->linear.z;
  twist.angular.x = gain_ * msg->angular.x;
  twist.angular.y = gain_ * msg->angular.y;
  twist.angular.z = gain_ * msg->angular.z;
  command_buffer_.writeFromNonRT(twist);
}

void TwistController ::reconfigureCallback(const twist_controller::TwistControllerConfig& config, uint32_t level)
{
  gain_ = config.twist_gain;
}
}  // namespace ros_controllers_cartesian

PLUGINLIB_EXPORT_CLASS(ros_controllers_cartesian::TwistController, controller_interface::ControllerBase)
