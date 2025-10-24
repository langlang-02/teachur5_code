#pragma once

#include <ros/ros.h>
#include <ur_dashboard_msgs/Load.h>
#include <ur_dashboard_msgs/GetRobotMode.h>
#include <ur_dashboard_msgs/RawRequest.h>
#include <ur_dashboard_msgs/GetProgramState.h>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <serial/serial.h>
#include <string>

// 用于ur5e
class dashboardsrv_client
{
public:
    dashboardsrv_client(ros::NodeHandle nh, std::string pf_name);
    /**
     * @brief 在示教板上载入特定名称的程序
     *
     * @param filename 程序名称
     */
    void load_program(const std::string &filename);
    /**
     * @brief 开始运行当前载入的程序,相当于在示教板按下播放键
     *
     */
    void play();
    /**
     * @brief 停止运行当前载入的程序,相当于在示教板按下停止键
     *
     */
    void stop();
    /**
     * @brief 启动机械臂,载入并运行remote_comtrol程序
     *
     */
    void robot_init();
    /**
     * @brief 初始化数字输出为0
     *
     */
    void DO_init();
    /**
     * @brief 设置数字输出
     *
     * @param pin 索引
     * @param state 高低电平
     */
    void setDO(int8_t pin, bool state);
    /**
     * @brief 接触保护性停止
     *
     */
    void unlockPS();
    /**
     * @brief 关机
     *
     */
    void power_off();

    void zeroFtSensor();
    void closePopUp();
    void closeSafetyPopUp();
private:
    ros::NodeHandle nh_;
    ros::ServiceClient srv_client_;
    ros::ServiceClient load_client_;
    ros::ServiceClient DO_client_;
    ros::ServiceClient program_client_;
    ros::ServiceClient safety_client_;
    ros::ServiceClient zero_ftsensor_client_;
    ros::ServiceClient close_safety_popup_client_;
    ros::ServiceClient close_popup_client_;

    std::string prefix_name;
};
