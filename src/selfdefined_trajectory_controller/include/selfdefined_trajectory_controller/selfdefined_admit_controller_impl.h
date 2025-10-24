#pragma once

#include <atomic>
std::atomic_flag spin_lock = ATOMIC_FLAG_INIT;

#include "selfdefined_trajectory_controller/selfdefined_trajectory_controller.h"

namespace selfdefined_controllers
{
  template <class SegmentImpl, class HardwareInterface>
  bool SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      init(HardwareInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    using namespace joint_trajectory_controller;
    using namespace joint_trajectory_controller::internal;

    // ------------0.获取控制器名称-----------------
    this->controller_nh_ = controller_nh;
    // 控制器名称selfdefined_trajectory_controller
    this->name_ = getLeafNamespace(this->controller_nh_);
    // 获得该控制器所处的机械臂名称
    this->robot_name_ = getParentNamespace(this->controller_nh_, 1);

    // ------------7.获取定义参数----------------ur_robot_driver的launch中设置
    // wrench的死区,小于死区则置为0
    controller_nh.param<std::vector<double>>("wrench_dead", wrenchDead_,
                                             std::vector<double>{0.5, 0.5, 0.5, 0.1, 0.1, 0.1});
    // 最大平移速度
    controller_nh.param<double>("tra_velmax", traVelLimit_, 1.0);
    // 最大旋转速度
    controller_nh.param<double>("rot_velmax", rotVelLimit_, 1.57);
    // 最大平移加速度
    controller_nh.param<double>("tra_accmax", traAccLimit_, 5.0);
    // 最大旋转加速度
    controller_nh.param<double>("rot_accmax", rotAccLimit_, 6.28);
    // 是否打印调试信息
    controller_nh.param<bool>("verbose", verbose_, false);

    // ------------8.获取该控制器所控制的关节信息----------------
    this->joint_names_ = getStrings(this->controller_nh_, "joints");
    if (this->joint_names_.empty())
      return false;
    const unsigned int n_joints = this->joint_names_.size();

    // ------------9.获取URDF模型----------------
    std::string robot_description;
    controller_nh.param<std::string>("robot_description", robot_description,
                                     "robot_description");
    urdf::ModelSharedPtr urdf = getUrdf(root_nh, robot_description);
    if (!urdf)
      return false;

    // ------------10.初始化pinocchio动力学库----------------
    // 10.1 从 URDF 模型构建 Pinocchio 机器人模型-----------
    pinocchio::urdf::buildModel(urdf, this->robot_model_);
    // 10.2 初始化三个 pinocchio::Data 对象-----------
    this->model_curr_data_ = pinocchio::Data(this->robot_model_);    // 机器人当前状态数据
    this->model_desired_data_ = pinocchio::Data(this->robot_model_); // 机器人期望状态数据
    this->model_temp_data_ = pinocchio::Data(this->robot_model_);    // 机器人暂时状态数据
    // 10.3 检查 compliance_link 是否存在于 URDF 模型中-----------
    std::string compliance_link;
    controller_nh.param<std::string>("compliance_link", compliance_link,
                                     "tool0");
    if (!robot_model_.existFrame(compliance_link))
    {
      const std::string error = "Failed to parse robot chain from urdf model. "
                                "Are you sure that compliance_link exists?";
      ROS_ERROR_STREAM(error);
      throw std::runtime_error(error);
    }

    // 10.4 获取 compliance_link 的 frame ID-----------
    this->compliance_frameId_ = robot_model_.getFrameId(compliance_link);
    // 10.5 获取 URDF 模型中的关节信息
    std::vector<urdf::JointConstSharedPtr> urdf_joints = getUrdfJoints(*urdf, this->joint_names_);
    if (urdf_joints.empty())
      return false;
    // 10.6 断言关节数量是否匹配-----------
    assert(n_joints == urdf_joints.size());
    // 10.7 初始化成员变量-----------
    // 1. 初始化 Eigen 向量，用于存储当前和期望的关节位置、速度和加速度
    this->pino_curr_pos_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_curr_vel_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_pos_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_vel_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_acc_ = Eigen::VectorXd::Zero(n_joints);
    // 2. 初始化一个 6 行, robot_model_.nv 列的 Eigen 矩阵，用于存储雅可比矩阵
    this->J_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    // 初始化目标、当前和误差的力矩（wrench），它们的初始值全部为零
    this->target_wrench_.setZero();
    this->current_wrench_.setZero();
    this->error_wrench_.setZero();
    // 3. 将 error_wrench_ 写入一个实时缓冲区（Real-Time Buffer），以确保数据的一致性和实时性
    error_wrench_rb_.writeFromNonRT(this->error_wrench_);
    // 4. 调整该类的 joints_ 和 angle_wraparound_ 向量的大小，以适应关节的数量
    this->joints_.resize(n_joints);
    this->angle_wraparound_.resize(n_joints);

    // 10.8 初始化循环，遍历所有关节，并执行相应操作-----------
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      // 1. 获取关节句柄：尝试从硬件接口获取关节句柄，并存储在 joints_ 向量中
      try
      {
        this->joints_[i] = hw->getHandle(this->joint_names_[i]);
        ROS_DEBUG_STREAM_NAMED(this->name_, "Successfully obtained handle for joint '"
                                                << this->joint_names_[i] << "'.");
      }
      catch (...)
      {
        ROS_ERROR_STREAM_NAMED(this->name_, "Could not find joint '"
                                                << this->joint_names_[i] << "' in '"
                                                << this->getHardwareInterfaceType() << "'.");
        return false;
      }
      // 2. 检查关节是否是连续的
      this->angle_wraparound_[i] =
          urdf_joints[i]->type == urdf::Joint::CONTINUOUS;
      const std::string not_if = this->angle_wraparound_[i] ? "" : "non-";
      // 3. 记录调试信息
      ROS_DEBUG_STREAM_NAMED(this->name_, "Found " << not_if << "continuous joint '"
                                                   << this->joint_names_[i] << "' in '"
                                                   << this->getHardwareInterfaceType() << "'.");
    }
    assert(this->joints_.size() == this->angle_wraparound_.size());
    // 4. 输出一些调试信息，包括控制器名称、关节数量、硬件接口类型和轨迹段类型
    ROS_DEBUG_STREAM_NAMED(this->name_, "Initialized controller '"
                                            << this->name_ << "' with:"
                                            << "\n- Number of joints: " << this->getNumberOfJoints()
                                            << "\n- Hardware interface type: '"
                                            << this->getHardwareInterfaceType() << "'");

    // 订阅目标wrench和当前wrench话题
    std::string topic_target_wrench;
    controller_nh.param<std::string>("topic_target_wrench", topic_target_wrench,
                                     "target_wrench");
    std::string topic_current_wrench;
    controller_nh.param<std::string>("topic_current_wrench", topic_current_wrench,
                                     "current_wrench");
    this->target_wrench_sub_ = controller_nh.subscribe(topic_target_wrench, 1, &SelfDefinedTrajectoryController::targetWrenchCB, this);
    this->current_wrench_sub_ = controller_nh.subscribe(topic_current_wrench, 1, &SelfDefinedTrajectoryController::currentWrenchCB, this);

    // 初始化动态配置服务器
    this->dyn_conf_server_ = std::make_unique<dynamic_reconfigure::Server<selfdefined_trajectory_controller::selfdefinedControllerConfig>>(controller_nh);
    dynamic_reconfigure::Server<selfdefined_trajectory_controller::selfdefinedControllerConfig>::CallbackType f;
    f = boost::bind(&SelfDefinedTrajectoryController::dynamicReconfigureCallback, this, _1, _2);
    this->dyn_conf_server_->setCallback(f);

    return true;
  }
}