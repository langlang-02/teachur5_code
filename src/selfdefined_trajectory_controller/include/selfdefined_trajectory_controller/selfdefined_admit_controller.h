#pragma once

// pinocchio库文件
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/QR>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include "selfdefined_trajectory_controller/selfdefinedControllerConfig.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#define EPSILON 1e-5

namespace selfdefined_controllers
{
  // 定义模板类SelfDefinedTrajectoryController，继承于
  // joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
  template <class SegmentImpl, class HardwareInterface>
  class SelfDefinedTrajectoryController
      : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
  {
  public:
    // 构造与析构均为默认
    SelfDefinedTrajectoryController() = default;
    virtual ~SelfDefinedTrajectoryController() = default;

    // 控制器初始化
    bool init(HardwareInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    // 控制器更新: 当前时刻time, 控制器更新周期2ms
    void update(const ros::Time &time, const ros::Duration &period);

  protected:
    typedef Eigen::Matrix<double, 6, 1> Vector6D;
    // ------------------------简化引用------------------------------------
    // 简化对基类的引用
    using Base = joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>;
    // Base::ActionServer相当于
    // joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>::ActionServer
    using typename Base::ActionServer;
    using typename Base::ActionServerPtr;
    using typename Base::GoalHandle;
    using typename Base::HwIfaceAdapter;
    using typename Base::JointHandle;
    using typename Base::JointTrajectoryConstPtr;
    using typename Base::RealtimeGoalHandle;
    using typename Base::RealtimeGoalHandlePtr;
    using typename Base::Scalar;
    using typename Base::Segment;
    using typename Base::StatePublisher;
    using typename Base::StatePublisherPtr;
    using typename Base::Trajectory;
    using typename Base::TrajectoryBox;
    using typename Base::TrajectoryPerJoint;
    using typename Base::TrajectoryPerJointPtr;
    using typename Base::TrajectoryPtr;
    // ------------------------简化引用------------------------------------

    // ---------------------控制算法功能函数--------------------------------
    void admittance_control(const ros::Duration &period);
    double error_zforce[6][3], dD_value[6];
    double AD_kp, AD_ki, AD_kd;
    double* Adaptive_Damp_Z(const ros::Duration &period);
    double Coor_para;
    // ---------------------控制算法功能函数--------------------------------

  private:
    std::string robot_name_;
    // ----------pinocchio动力学库的数据成员----------
    // "表现"出柔顺性质的关节id
    pinocchio::FrameIndex compliance_frameId_;
    // 机械臂模型的当前\期望和临时数据
    pinocchio::Data model_curr_data_, model_desired_data_, model_temp_data_;
    // 机器人模型
    pinocchio::Model robot_model_;
    // 当前位姿和期望位姿
    pinocchio::SE3 curr_pose_, desired_pose_;
    // 当前速度,期望速度,期望加速度
    pinocchio::Motion curr_vel_, desired_vel_, desired_acc_;
    // 位姿误差,速度误差,加速度误差
    pinocchio::Motion x_e_, dx_e_, ddx_e_;
    // ----------pinocchio动力学库的数据成员----------

    // --------------运动限制参数----------------
    // 是否打印调试内容
    bool verbose_;
    // wrench 死区
    std::vector<double> wrenchDead_;
    // 平移速度最大限制 traslation velocity limit
    double traVelLimit_;
    // 旋转速度最大限制 rotation velocity limit
    double rotVelLimit_;
    // 平移加速度最大限制
    double traAccLimit_;
    // 旋转加速度最大限制
    double rotAccLimit_;
    // --------------运动限制参数----------------

    // ----------------------导纳控制器参数------------------------
    // 雅克比矩阵
    Eigen::MatrixXd J_;
    // "表现"出柔顺性质的关节的名称
    std::string compliance_link_;
    // 导纳控制中的M,D,K矩阵
    Eigen::Matrix<double, 6, 6> M_, D_, K_;
    // 当前关节角和关节角速度
    Eigen::VectorXd pino_curr_pos_, pino_curr_vel_;
    // 期望关节角,关节角速度和关节角加速度
    Eigen::VectorXd pino_desired_pos_, pino_desired_vel_, pino_desired_acc_;

    // ---------------动态配置参数----------------
    // 动态参数配置服务器,用于动态调整导纳控制中的M,D,K
    std::unique_ptr<dynamic_reconfigure::Server<
        selfdefined_trajectory_controller::selfdefinedControllerConfig>>
        dyn_conf_server_;
    void dynamicReconfigureCallback(
        selfdefined_trajectory_controller::selfdefinedControllerConfig &config, uint32_t level);
    // ---------------动态配置参数----------------
    // ----------------------导纳控制器参数------------------------

    // ------------自定义功能函数参数----------------
    // 用于订阅 目标wrench 和 当前wrench 的subscriber
    ros::Subscriber target_wrench_sub_, current_wrench_sub_;
    // 目标wrench 和 当前wrench 的误差
    Vector6D error_wrench_;
    // 当前wrench
    pinocchio::Force current_wrench_;
    // 目标wrench
    pinocchio::Force target_wrench_;
    // 实时buffer
    realtime_tools::RealtimeBuffer<Vector6D> error_wrench_rb_;
    // ------------自定义功能函数参数----------------
  };
} // namespace selfdefined_controllers

#include "selfdefined_trajectory_controller/selfdefined_trajectory_controller_impl.h"