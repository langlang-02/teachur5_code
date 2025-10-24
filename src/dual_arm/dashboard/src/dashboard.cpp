#include <dashboard/dashboard.h>

// dashboardsrv_client::dashboardsrv_client(ros::NodeHandle nh)
dashboardsrv_client::dashboardsrv_client(ros::NodeHandle nh, std::string pf_name)
{
  nh_ = nh;
  prefix_name = pf_name;

  bool no_serial;
  std::string soft_gripper;
  srv_client_ = nh_.serviceClient<ur_dashboard_msgs::RawRequest>(prefix_name + "/ur_hardware_interface/dashboard/raw_request");
  load_client_ = nh_.serviceClient<ur_dashboard_msgs::Load>(prefix_name + "/ur_hardware_interface/dashboard/load_program");
  DO_client_ = nh_.serviceClient<ur_msgs::SetIO>(prefix_name + "/ur_hardware_interface/set_io");
  program_client_ = nh_.serviceClient<ur_dashboard_msgs::GetProgramState>(prefix_name + "/ur_hardware_interface/dashboard/program_state");
  safety_client_ = nh_.serviceClient<std_srvs::Trigger>(prefix_name + "/ur_hardware_interface/dashboard/unlock_protective_stop");
  zero_ftsensor_client_ = nh_.serviceClient<std_srvs::Trigger>(prefix_name + "/ur_hardware_interface/zero_ftsensor");
  close_safety_popup_client_ = nh_.serviceClient<std_srvs::Trigger>(prefix_name + "/ur_hardware_interface/dashboard/close_safety_popup");
  close_popup_client_ = nh_.serviceClient<std_srvs::Trigger>(prefix_name + "/ur_hardware_interface/dashboard/close_popup");

}
void dashboardsrv_client::DO_init()
{
  setDO(0, false);
  setDO(1, false);
  setDO(2, false);
  setDO(3, false);
}

void dashboardsrv_client::play()
{
  ur_dashboard_msgs::RawRequest play;
  play.request.query = "play\n";
  size_t i = 0;
  while(1)
  // for (; i < 5; ++i)
  {
    if (!ros::topic::waitForMessage<std_msgs::Bool>(prefix_name + "/ur_hardware_interface/robot_program_running")->data)
    {
      ros::Duration(0.5).sleep();
      srv_client_.call(play);
    }
    else{
      ROS_WARN_STREAM(prefix_name + ": 启动remote_control程序成功");
      return;
    }
  }
  ROS_WARN_STREAM(prefix_name + ": 尝试启动remote_control程序失败,请手动操作示教板");
}

void dashboardsrv_client::stop()
{
  ur_dashboard_msgs::RawRequest stop;
  stop.request.query = "stop\n";
  for (size_t i = 0; i < 5; ++i)
  {
    if (ros::topic::waitForMessage<std_msgs::Bool>(prefix_name + "/ur_hardware_interface/robot_program_running")->data)
    {
      ros::Duration(0.5).sleep();
      srv_client_.call(stop);
    }
    else{
      ROS_WARN_STREAM(prefix_name + ": 停止remote_control程序成功");
      return;
    }
  }
  ROS_WARN_STREAM(prefix_name + ": 尝试停止remote_control程序失败,请手动操作示教板");
}

void dashboardsrv_client::unlockPS()
{
  std_srvs::Trigger unlock;
  size_t num = 10;
  for (size_t i = 0; i < num; ++i)
  {
    if (ros::topic::waitForMessage<ur_dashboard_msgs::SafetyMode>(prefix_name + "/ur_hardware_interface/safety_mode")->mode == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP)
    {
      ros::Duration(1).sleep();
      safety_client_.call(unlock);
    }
    else{
      ROS_WARN_STREAM(prefix_name + ": 解除保护性停止成功");
      return;
    }
  }
  ROS_WARN_STREAM(prefix_name + ": 尝试解除保护性停止失败,请手动操作示教板");
}
void dashboardsrv_client::zeroFtSensor()
{
  std_srvs::Trigger zero;
  zero_ftsensor_client_.call(zero);
  if(zero.response.success)
    ROS_INFO_STREAM(prefix_name + ": 力传感器调零成功");
  else
    ROS_WARN_STREAM(prefix_name + ": 力传感器调零失败,请检查力传感器连接或机械臂版本(e系列)");
}
void dashboardsrv_client::closePopUp()
{
  std_srvs::Trigger close;
  close_popup_client_.call(close);
  if(close.response.success)
    ROS_INFO_STREAM(prefix_name + ": 关闭弹窗成功");
  else
    ROS_WARN_STREAM(prefix_name + ": 关闭弹窗失败,请手动操作示教板");
}
void dashboardsrv_client::closeSafetyPopUp()
{
  std_srvs::Trigger close;
  close_safety_popup_client_.call(close);
  if(close.response.success)
    ROS_INFO_STREAM(prefix_name + ": 关闭安全弹窗成功");
  else
    ROS_WARN_STREAM(prefix_name + ": 关闭安全弹窗失败,请手动操作示教板");
}

void dashboardsrv_client::load_program(const std::string &filename)
{
  ROS_INFO_STREAM(prefix_name + "等待" + filename.substr(0, filename.size() - 1) + "加载完成");
  auto programState = ros::topic::waitForMessage<std_msgs::Bool>(prefix_name + "/ur_hardware_interface/robot_program_running");
  if (!programState->data)
  {
    // this->stop();
    ur_dashboard_msgs::Load load;
    load.request.filename = std::string(filename);
    load_client_.call(load);
    play();
    while (!ros::topic::waitForMessage<std_msgs::Bool>(prefix_name + "/ur_hardware_interface/robot_program_running")->data)
    {
      ros::Duration(1).sleep();
    }
  }
}

void dashboardsrv_client::setDO(int8_t pin, bool state)
{
  ur_msgs::SetIO srv;
  srv.request.fun = srv.request.FUN_SET_DIGITAL_OUT;
  srv.request.pin = pin;
  srv.request.state = state ? srv.request.STATE_ON : srv.request.STATE_OFF;
  DO_client_.call(srv);
}

void dashboardsrv_client::robot_init()
{
  ur_dashboard_msgs::RawRequest init;
  ur_dashboard_msgs::RobotModeConstPtr robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>(prefix_name + "/ur_hardware_interface/robot_mode");
  if (robotmode->mode == robotmode->RUNNING)
  {
    ROS_INFO_STREAM(prefix_name + ": Robot is RUNNING\n");
    return;
  }
  ROS_INFO_STREAM(prefix_name + " Robot is Initializing\n");
  init.request.query = "power on\n";
  srv_client_.call(init);
  init.request.query = "brake release\n";
  srv_client_.call(init);
  closePopUp();
  closeSafetyPopUp();
  while (robotmode->mode != robotmode->RUNNING)
  {
    robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>(prefix_name + "/ur_hardware_interface/robot_mode");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO_STREAM(prefix_name + ": Robot is RUNNING\n");
}

void dashboardsrv_client::power_off()
{
  ur_dashboard_msgs::RawRequest init;
  ur_dashboard_msgs::RobotModeConstPtr robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>(prefix_name + "/ur_hardware_interface/robot_mode");
  if (robotmode->mode == robotmode->POWER_OFF)
  {
    ROS_INFO_STREAM(prefix_name + ": Robot is CLOSING\n");
    return;
  }
  ROS_INFO_STREAM(prefix_name + ": Robot is Power off\n");
  init.request.query = "power off\n";
  srv_client_.call(init);

  while (robotmode->mode != robotmode->POWER_OFF)
  {
    robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>(prefix_name + "/ur_hardware_interface/robot_mode");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO_STREAM(prefix_name + ": Robot is CLOSING\n");
}

