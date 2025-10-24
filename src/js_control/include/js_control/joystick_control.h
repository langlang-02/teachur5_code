#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager_msgs/SwitchController.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <dashboard/dashboard.h>
class F710
{
public:
    F710();
    int x, a, b, y, LB, RB, LT, RT, BACK, unknown1, unknown2, unknown3;
    float button_l, button_up, rstick_up, rstick_l, lstick_up, lstick_l;
};

class ur5_js
{
public:
    ur5_js(){}
    ur5_js(ros::NodeHandle nh,
           std::string robot_name,
           std::string tool_name,
           tf::Matrix3x3 world_to_base_);
    ~ur5_js();
    void add_end(std::string end_name);
    tf::Vector3 get_abs_end_to_tool_vector(void);
    void run();

private:
    std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> _realtime_pub;
    ros::Subscriber _sub,twist_sub;
    ros::ServiceClient _srv;
    std::unique_ptr<F710> _joystick_ptr;
    void Callback1(const sensor_msgs::Joy::ConstPtr joy_cmd);
    void twistMoveCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    double _tra_scale, _rot_scale;
    std::string robot_name,tool_name,twist_topic_name;
    std::vector<std::string> end_names;
    std::vector<std::string>::iterator curr_end;
    tf::Matrix3x3 world_to_base;
    tf::Vector3 twist_world_linear_velocity,twist_world_angular_velocity;
    int target_control_mode;//1表示收到twist参数并持续运动，0表示收到twist结束运动参数，-1表示无twist运动参数
    ros::NodeHandle nh;
    std::shared_ptr<dashboardsrv_client> robot_dbptr;
};
void takeControl();
void freeControl();