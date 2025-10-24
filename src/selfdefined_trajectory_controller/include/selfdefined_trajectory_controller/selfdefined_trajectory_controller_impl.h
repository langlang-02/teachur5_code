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

    // ------------1.设置控制器发布更新频率----------------ur_robot_driver的launch中设置300
    double state_publish_rate = 50.0;
    this->controller_nh_.getParam("state_publish_rate", state_publish_rate);
    ROS_DEBUG_STREAM_NAMED(this->name_, "Controller state will be published at "
                                            << state_publish_rate << "Hz.");
    // 设置发布频率50Hz
    this->state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

    // ------------2.设置动作监视更新频率------------------ur_robot_driver的launch中设置20
    double action_monitor_rate = 20.0;
    this->controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
    ROS_DEBUG_STREAM_NAMED(this->name_, "Action status changes will be monitored at "
                                            << action_monitor_rate << "Hz.");
    // 设置监视频率20Hz
    this->action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);

    // ------------3.设置停止轨迹的持续时间----------------在ur_robot_driver的launch中设置0.5
    double stop_trajectory_duration = 0.0;
    this->controller_nh_.getParam("stop_trajectory_duration", stop_trajectory_duration);
    ROS_DEBUG_STREAM_NAMED(this->name_, "Stop trajectory has a duration of "
                                            << stop_trajectory_duration << "s.");
    this->stop_trajectory_duration_ = stop_trajectory_duration;

    // ------------4.检查是否allow_partial_joints_goal_设置----------------目前设置为false
    controller_nh.param<bool>("allow_partial_joints_goal",
                              this->allow_partial_joints_goal_, false);
    // 如果允许部分关节目标，则输出调试信息
    if (this->allow_partial_joints_goal_)
    {
      ROS_DEBUG_NAMED(this->name_, "Goals with partial set of joints are allowed");
    }

    // ------------5.获取目标力矩/当前力矩话题名称----------------
    // subscribed target wrench topic name
    std::string topic_target_wrench;
    controller_nh.param<std::string>("topic_target_wrench", topic_target_wrench,
                                     "target_wrench");
    ROS_DEBUG_STREAM_NAMED(this->name_, "Controller target wrench will be subscribed at "
                                            << topic_target_wrench);
    // subscribed current wrench topic name
    std::string topic_current_wrench;
    controller_nh.param<std::string>("topic_current_wrench", topic_current_wrench,
                                     "current_wrench");
    ROS_INFO_STREAM_NAMED(this->name_, "Controller current wrench will be subscribed at "
                                           << topic_current_wrench);

    std::string topic_currel_errorpose;
    controller_nh.param<std::string>("topic_currel_errorpose", topic_currel_errorpose,
                                     "error_pos_otherToself");
    ROS_INFO_STREAM_NAMED(this->name_, "Controller current pose relerror will be subscribed at "
                                           << topic_currel_errorpose);

    // ------------6.获取所作用的柔顺关节----------------right_tool0或left_tool0
    controller_nh.param<std::string>("compliance_link", this->compliance_link_,
                                     "tool0");
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
    if (!robot_model_.existFrame(compliance_link_))
    {
      const std::string error = "Failed to parse robot chain from urdf model. "
                                "Are you sure that compliance_link exists?";
      ROS_ERROR_STREAM(error);
      throw std::runtime_error(error);
    }

    // 10.4 获取 compliance_link 的 frame ID-----------
    this->compliance_frameId_ = robot_model_.getFrameId(compliance_link_);
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
    /////////////////////////////
    for (size_t i = 0; i < 6; i++)
      for(size_t j = 0; j < 3; j++)
          error_zforce[i][j] = 0.0;
    /////////////////////////////
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
                                            << this->getHardwareInterfaceType() << "'"
                                            << "\n- Trajectory segment type: '"
                                            << hardware_interface::internal::demangledTypeName<SegmentImpl>() << "'");

    // ------------11.设置公差----------------
    ros::NodeHandle tol_nh(this->controller_nh_, "constraints");
    this->default_tolerances_ = getSegmentTolerances<typename Base::Scalar>(tol_nh, this->joint_names_);

    // ------------12.初始化硬件接口适配器----------------
    this->hw_iface_adapter_.init(this->joints_, this->controller_nh_);

    // ------------13.创建话题订阅----------------
    // 1. 参考轨迹订阅：/selfdefined_trajectory_controller/command
    this->trajectory_command_sub_ = this->controller_nh_.subscribe(
        "command", 1, &SelfDefinedTrajectoryController::trajectoryCommandCB, this);
    // ----- 双臂参考轨迹话题
    this->dual_trajectory_command_sub_ = this->controller_nh_.subscribe(
        "/dual_robot/selfdefined_trajectory_controller/command", 1,
        &SelfDefinedTrajectoryController::dualtrajectoryCommandCB, this);
    // 2. 订阅 当前wrench 的topic
    this->current_wrench_sub_ = this->controller_nh_.subscribe(topic_current_wrench, 1,
                                                               &SelfDefinedTrajectoryController::currentWrenchCB, this,
                                                               ros::TransportHints().reliable().tcpNoDelay()); // 传输提示，设置传输为可靠并禁用延迟
    // 3. 订阅 期望wrench 的topic
    this->target_wrench_sub_ = this->controller_nh_.subscribe(topic_target_wrench, 1,
                                                              &SelfDefinedTrajectoryController::targetWrenchCB, this);

    // 4. 订阅 当前error_RtoL的topic
    this->currel_errorpose_sub_ = this->controller_nh_.subscribe(topic_currel_errorpose, 1,
                                                                 &SelfDefinedTrajectoryController::currentRelerrorposCB, this,
                                                                 ros::TransportHints().reliable().tcpNoDelay()); // 传输提示，设置传输为可靠并禁用延迟

    // ------------14.创建话题发布----------------
    // 1. 创建新的 StatePublisher 对象，用于发布控制器的状态信息
    this->state_publisher_.reset(new StatePublisher(this->controller_nh_, "state", 1));
    // 2. 创建新的 ActionServer 对象，用于处理动作目标和取消请求
    this->action_server_.reset(new ActionServer(this->controller_nh_, "follow_joint_trajectory",
                                                std::bind(&SelfDefinedTrajectoryController::goalCB, this, std::placeholders::_1),
                                                std::bind(&SelfDefinedTrajectoryController::cancelCB, this, std::placeholders::_1),
                                                false));
    // 3. 启动动作服务器，开始处理动作请求
    this->action_server_->start();
    // 4. 创建并广告 服务 query_state，用于查询控制器的状态
    this->query_state_service_ = this->controller_nh_.advertiseService(
        "query_state", &SelfDefinedTrajectoryController::queryStateService, this);

    // ------------15.预分配资源----------------
    // 1. 存储不同状态下的数据
    this->current_state_ = typename Segment::State(n_joints);
    this->old_desired_state_ = typename Segment::State(n_joints);
    this->desired_state_ = typename Segment::State(n_joints);
    this->state_error_ = typename Segment::State(n_joints);
    this->desired_joint_state_ = typename Segment::State(1);
    this->state_joint_error_ = typename Segment::State(1);
    // 2. 成功的关节轨迹
    this->successful_joint_traj_ = boost::dynamic_bitset<>(this->getNumberOfJoints());
    // 3. 创建和检查保持轨迹
    this->hold_trajectory_ptr_ = this->createHoldTrajectory(n_joints);
    assert(this->joint_names_.size() == this->hold_trajectory_ptr_->size());

    // ------------16.选择不同的 TrajectoryBuilder 类型----------------
    if (this->stop_trajectory_duration_ == 0.0)
    { // 控制器会生成一个保持当前状态的轨迹
      this->hold_traj_builder_ = std::unique_ptr<TrajectoryBuilder<SegmentImpl>>(
          new HoldTrajectoryBuilder<SegmentImpl, HardwareInterface>(this->joints_));
    }
    else
    { // 控制器会生成一个停止的轨迹, 该轨迹将在 stop_trajectory_duration_ 内将关节状态减速到停止
      this->hold_traj_builder_ = std::unique_ptr<TrajectoryBuilder<SegmentImpl>>(
          new StopTrajectoryBuilder<SegmentImpl>(this->stop_trajectory_duration_, this->desired_state_));
    }

    // ------------17.设置动态配置服务器,用于动态调整导纳控制器中的M,D,K三个矩阵的参数------------
    dyn_conf_server_.reset(new dynamic_reconfigure::Server<
                           selfdefined_trajectory_controller::selfdefinedControllerConfig>(this->controller_nh_));
    dyn_conf_server_->setCallback(
        std::bind(&SelfDefinedTrajectoryController::dynamicReconfigureCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    // --------------------------------------------------------------------------------------

    // ------------18.初始化了状态消息的各个字段，并在完成修改后解锁发布器----------------
    {
      this->state_publisher_->lock();
      this->state_publisher_->msg_.joint_names = this->joint_names_;
      this->state_publisher_->msg_.desired.positions.resize(n_joints);
      this->state_publisher_->msg_.desired.velocities.resize(n_joints);
      this->state_publisher_->msg_.desired.accelerations.resize(n_joints);
      this->state_publisher_->msg_.actual.positions.resize(n_joints);
      this->state_publisher_->msg_.actual.velocities.resize(n_joints);
      this->state_publisher_->msg_.error.positions.resize(n_joints);
      this->state_publisher_->msg_.error.velocities.resize(n_joints);
      this->state_publisher_->unlock();
    }
    return true;
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      update(const ros::Time &time, const ros::Duration &period)
  {
    // ------------1.获取当前需要执行的轨迹----------------
    // typename关键字用于指定Base类的成员类型TrajectoryPtr，以确保编译器正确解析该类型
    typename Base::TrajectoryPtr curr_traj_ptr;
    this->curr_trajectory_box_.get(curr_traj_ptr);
    typename Base::Trajectory &curr_traj = *curr_traj_ptr;

    // ------------2.更新时间数据----------------（以便在控制器运行期间保持最新的时间信息和控制周期）
    typename Base::TimeData time_data;
    time_data.time = time;                                                       // 缓存当前时间
    time_data.period = ros::Duration(period.toSec());                            // 缓存当前控制周期2ms
    time_data.uptime = this->time_data_.readFromRT()->uptime + time_data.period; // 更新控制器运行时间
    this->time_data_.writeFromNonRT(time_data);                                  // 写入新的时间数据

    // ------------3.更新当前状态与状态误差---------------
    for (unsigned int i = 0; i < this->joints_.size(); ++i)
    {
      // 3.1 使用上一周期的控制关节状态作为当前关节状态（位置，速度，(加速度)）(末端笛卡尔空间)
      this->current_state_.position[i] = this->desired_state_.position[i];
      this->pino_curr_pos_(i) = this->desired_state_.position[i];
      this->current_state_.velocity[i] = this->desired_state_.velocity[i];
      this->pino_curr_vel_(i) = this->desired_state_.velocity[i];
      // 3.2 获取第i个关节的期望状态（位置，速度，加速度）(关节空间)
      // 赋值给desired_joint_state_, typename Segment::State(1);
      typename Base::TrajectoryPerJoint::const_iterator segment_it = sample(
          curr_traj[i], time_data.uptime.toSec(), this->desired_joint_state_);
      if (curr_traj[i].end() == segment_it)
      {
        // 如果在当前时间没有定义轨迹，记录错误日志
        ROS_ERROR_NAMED(this->name_,
                        "Unexpected error: No trajectory defined at current "
                        "time. Please contact the package maintainer.");
        return;
      }
      // 3.4 获得当前时刻的理论关节状态pino_desired_
      this->pino_desired_pos_(i) = this->desired_joint_state_.position[0];
      this->pino_desired_vel_(i) = this->desired_joint_state_.velocity[0];
      this->pino_desired_acc_(i) = this->desired_joint_state_.acceleration[0];
      // 3.5 比较期望关节与上一周期的控制关节状态的差值
      // 位置误差通过shortest_angular_distance来计算，以确保角度误差在[-π, π]范围内
      this->state_error_.position[i] = angles::shortest_angular_distance(
          this->current_state_.position[i], this->desired_joint_state_.position[0]);
      this->state_error_.velocity[i] =
          this->desired_joint_state_.velocity[0] - this->current_state_.velocity[i];
      this->state_error_.acceleration[i] = 0.0;
      // 3.6 检查是否到达轨迹执行时间
      const typename Base::RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
      if (rt_segment_goal && rt_segment_goal == this->rt_active_goal_)
      {
        if (segment_it == --curr_traj[i].end())
        {
          const ros::Time uptime = this->time_data_.readFromRT()->uptime;
          if (uptime.toSec() < segment_it->endTime())
            ;
          else
            this->successful_joint_traj_[i] = 1;
        }
      }
    }

    // ------------4.通过更新pinocchio获得机器人当前状态---------------
    // 4.1 更新机器人当前状态
    pinocchio::forwardKinematics(robot_model_, model_curr_data_,
                                 pino_curr_pos_, pino_curr_vel_);
    pinocchio::updateFramePlacements(robot_model_, model_curr_data_);
    pino_rstate_rb_.writeFromNonRT(model_curr_data_);
    // 4.2 获得compliance_link_的实际位置（相对于bozi_link？)
    curr_pose_ = model_curr_data_.oMf[compliance_frameId_]; // pinocchio::SE3
    // 4.3 获得compliance_link_的实际末端速度
    curr_vel_ = pinocchio::getFrameVelocity(robot_model_, model_curr_data_,
                                            compliance_frameId_,
                                            pinocchio::LOCAL_WORLD_ALIGNED); // pinocchio::Motion

    // ------------5.通过更新pinocchio获得机器人期望状态---------------
    // 5.1 更新机器人期望状态
    pinocchio::forwardKinematics(robot_model_, model_desired_data_,
                                 pino_desired_pos_, pino_desired_vel_,
                                 pino_desired_acc_);
    // 5.2 获得compliance_link_的期望位置，相对于base_link)
    desired_pose_ = pinocchio::updateFramePlacement(robot_model_,
                                                    model_desired_data_,
                                                    compliance_frameId_); // pinocchio::SE3
    // 5.3 获得compliance_link_的期望末端速度
    desired_vel_ = pinocchio::getFrameVelocity(robot_model_, model_desired_data_,
                                               compliance_frameId_,
                                               pinocchio::LOCAL_WORLD_ALIGNED); // pinocchio::Motion
    // 5.3 获得compliance_link_的期望末端加速度
    desired_acc_ = pinocchio::getFrameAcceleration(robot_model_, model_desired_data_,
                                                   compliance_frameId_,
                                                   pinocchio::LOCAL_WORLD_ALIGNED); // pinocchio::Motion

    ////////////////////////////////控制器算法//////////////////////////////////////////
    admittance_control(period);
    //////////////////////////////////////////////////////////////////////////////////

    // ------------10.打印控制器的相关参数--------------
    if (verbose_)
    {
      ROS_INFO_STREAM_THROTTLE(1, "curr_q: " << std::endl
                                             << pino_curr_pos_.transpose());
      ROS_INFO_STREAM_THROTTLE(1, "desired_q:" << std::endl
                                               << pino_desired_pos_.transpose());
      ROS_INFO_STREAM_THROTTLE(1, "curr_qv: " << std::endl
                                              << pino_curr_vel_.transpose());
      ROS_INFO_STREAM_THROTTLE(1, "desired_qv:" << std::endl
                                                << pino_desired_vel_.transpose());
      ROS_INFO_STREAM_THROTTLE(1, "x_e_:" << std::endl
                                          << x_e_.toVector().transpose() << std::endl);
      ROS_INFO_STREAM_THROTTLE(1, "dx_e_:" << std::endl
                                           << dx_e_.toVector().transpose() << std::endl);
      ROS_INFO_STREAM_THROTTLE(1, "ddx_e_:" << std::endl
                                            << ddx_e_.toVector().transpose() << std::endl);
      ROS_INFO_STREAM_THROTTLE(1, "error_wrench:" << std::endl
                                                  << " f= " << error_wrench_.topRows(3).transpose() << std::endl
                                                  << " t = " << error_wrench_.bottomRows(3).transpose());
    }

    // ------------11.检查当前是否有活动目标--------------
    typename Base::RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
    if (current_active_goal &&
        this->successful_joint_traj_.count() == this->joints_.size())
    {
      current_active_goal->preallocated_result_->error_code =
          control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
      // 清理相关资源，确保不再发布反馈信息，并重置相关变量
      current_active_goal.reset(); // do not publish feedback
      this->rt_active_goal_.reset();
      this->successful_joint_traj_.reset();
    }

    // ------------12.硬件接口适配器：生成并发送命令--------------
    this->hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                          this->desired_state_,
                                          this->state_error_);

    // ------------13.帮助上层系统或用户界面实时监控控制器的状态--------------
    // 检查当前是否有活动目标
    if (current_active_goal)
    {
      // 设置反馈信息的时间戳
      current_active_goal->preallocated_feedback_->header.stamp = this->time_data_.readFromRT()->time;
      // 设置期望状态的反馈信息
      current_active_goal->preallocated_feedback_->desired.positions = this->desired_state_.position;
      current_active_goal->preallocated_feedback_->desired.velocities = this->desired_state_.velocity;
      current_active_goal->preallocated_feedback_->desired.accelerations = this->desired_state_.acceleration;
      // 设置实际状态的反馈信息
      current_active_goal->preallocated_feedback_->actual.positions = this->current_state_.position;
      current_active_goal->preallocated_feedback_->actual.velocities = this->current_state_.velocity;
      // 设置状态误差的反馈信息
      current_active_goal->preallocated_feedback_->error.positions = this->state_error_.position;
      current_active_goal->preallocated_feedback_->error.velocities = this->state_error_.velocity;
      // 发送反馈信息
      current_active_goal->setFeedback(current_active_goal->preallocated_feedback_);
    }

    // ------------14.发布系统的当前时间状态--------------
    this->publishState(time_data.uptime);
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      currentWrenchCB(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    /////////////////------单臂独立模式------//////////////////////////
    // ------------1.死区设置排除扰动项并赋值给current_wrench_--------------
    current_wrench_.linear()(0) =
        abs(msg->wrench.force.x) < wrenchDead_[0] ? 0 : msg->wrench.force.x;
    current_wrench_.linear()(1) =
        abs(msg->wrench.force.y) < wrenchDead_[1] ? 0 : msg->wrench.force.y;
    current_wrench_.linear()(2) =
        abs(msg->wrench.force.z) < wrenchDead_[2] ? 0 : msg->wrench.force.z;
    current_wrench_.angular()(0) =
        abs(msg->wrench.torque.x) < wrenchDead_[3] ? 0 : msg->wrench.torque.x;
    current_wrench_.angular()(1) =
        abs(msg->wrench.torque.y) < wrenchDead_[4] ? 0 : msg->wrench.torque.y;
    current_wrench_.angular()(2) =
        abs(msg->wrench.torque.z) < wrenchDead_[5] ? 0 : msg->wrench.torque.z;

    // 力位混合——研磨操作
    if (this->robot_name_ == "right_robot")
    {
      // 右手研磨，只关注z轴伸缩
      current_wrench_.linear()(0) = 0;
      current_wrench_.linear()(1) = 0;
      current_wrench_.angular()(0) = 0;
      current_wrench_.angular()(1) = 0;
      current_wrench_.angular()(2) = 0;
    }
    else if(this->robot_name_ == "left_robot")
    {
      // 左手辅助，关注平面研钵的平移，目前加持是xoz面
      // current_wrench_.angular()(0) = 0;
      // current_wrench_.angular()(1) = 0;
      // current_wrench_.angular()(2) = 0;
    }
    /////////////////------单臂独立模式------//////////////////////////

    // /////////////////------双臂协作模式------//////////////////////////
    // // 力位混合——研磨操作(该模式，力矩信息已经置换成world坐标系下，且力矩均为right_wrench)
    // if (this->robot_name_ == "right_robot")
    // {
    //   // 右手研磨，只关注z轴伸缩
    //   current_wrench_.linear()(0) = (1-Coor_para/100.0) * msg->wrench.force.x;
    //   current_wrench_.linear()(1) = (1-Coor_para/100.0) * msg->wrench.force.y;
    //   current_wrench_.linear()(2) = (1-0) * msg->wrench.force.z;
    //   current_wrench_.angular()(0) = msg->wrench.torque.x;
    //   current_wrench_.angular()(1) = msg->wrench.torque.y;
    //   current_wrench_.angular()(2) = msg->wrench.torque.z;
    // }
    // else if(this->robot_name_ == "left_robot")
    // {
    //   // 左手辅助，关注平面研钵的平移，目前加持是xoz面
    //   current_wrench_.linear()(0) = (-1*Coor_para/100.0) * msg->wrench.force.x;
    //   current_wrench_.linear()(1) = (-1*Coor_para/100.0) * msg->wrench.force.y;
    //   current_wrench_.linear()(2) = (1-1) * msg->wrench.force.z;
    //   current_wrench_.angular()(0) = msg->wrench.torque.x;
    //   current_wrench_.angular()(1) = msg->wrench.torque.y;
    //   current_wrench_.angular()(2) = msg->wrench.torque.z;
    // }
    // /////////////////------双臂协作模式------//////////////////////////

    // ------------2.从实时数据缓冲区读取机械臂当前状态信息--------------
    model_temp_data_ = *(pino_rstate_rb_.readFromRT());

    // ------------3.计算力矩误差--------------
    // 3.1 转换wrench的参考坐标系为base_link
    // current_wrench_在frame_id上
    // model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].act将current_wrench_转化到base_link中
    current_wrench_ = model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].act(current_wrench_);
    // std::cout << "current_wrench: " << current_wrench_ << std::endl;
    // 3.2 使用自旋锁保护共享资源
    while (spin_lock.test_and_set(std::memory_order_acquire))
      ;
    // 3.3 计算力矩误差，并保存在error_wrench_rb_中，方便update函数中调用
    error_wrench_rb_.writeFromNonRT(current_wrench_ - target_wrench_);
    // 3.4 释放自旋锁
    spin_lock.clear(std::memory_order_release);
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      currentRelerrorposCB(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    //////////////////////////////////////////////////////
    tf2::Quaternion quaternion(msg->pose.orientation.x,
                               msg->pose.orientation.y,
                               msg->pose.orientation.z,
                               msg->pose.orientation.w);
    double roll, pitch, yaw;
    // 欧拉角X(roll)   Y(pitch)    Z(yaw)
    tf2::Matrix3x3(quaternion).getEulerYPR(yaw, pitch, roll);

    current_wrench_.linear()(0) = -1 * msg->pose.position.x*500;
    current_wrench_.linear()(1) = -1 * msg->pose.position.y*500;
    current_wrench_.linear()(2) = -1 * msg->pose.position.z*500;
    current_wrench_.angular()(0) = -1 * roll * 500;
    current_wrench_.angular()(1) = -1 * pitch * 500;
    current_wrench_.angular()(2) = -1 * yaw * 500;

    // ------------2.从实时数据缓冲区读取机械臂当前状态信息--------------
    model_temp_data_ = *(pino_rstate_rb_.readFromRT());

    // ------------3.计算力矩误差--------------
    // 3.1 转换wrench的参考坐标系为base_link
    // current_wrench_在frame_id上
    // model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].act将current_wrench_转化到base_link中
    current_wrench_ = model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].act(current_wrench_);
    // std::cout << "current_wrench: " << current_wrench_ << std::endl;
    // 3.2 使用自旋锁保护共享资源
    while (spin_lock.test_and_set(std::memory_order_acquire))
      ;
    // 3.3 计算力矩误差，并保存在error_wrench_rb_中，方便update函数中调用
    error_wrench_rb_.writeFromNonRT(current_wrench_);
    // 3.4 释放自旋锁
    spin_lock.clear(std::memory_order_release);

    //////////////////////////////////////////////////////

    // // 提取目标帧位姿 (geometry_msgs::PoseStamped -> pinocchio::SE3)
    // Eigen::Vector3d position(msg->pose.position.x,
    //                          msg->pose.position.y,
    //                          msg->pose.position.z);

    // Eigen::Quaterniond quaternion(msg->pose.orientation.w,
    //                               msg->pose.orientation.x,
    //                               msg->pose.orientation.y,
    //                               msg->pose.orientation.z);

    // currel_errorpose_ = pinocchio::SE3(quaternion.toRotationMatrix(), position);

    // // ------------2.从实时数据缓冲区读取机械臂当前状态信息--------------
    // model_temp_data_ = *(pino_rstate_rb_.readFromRT());

    // // ------------3.计算相对位姿误差--------------
    // // 3.1 转换wrench的参考坐标系为base_link
    // // currel_errorpose_在frame_id上
    // // model_temp_data_.oMf[msg->header.frame_id] *将current_wrench_转化到world中
    // currel_errorpose_ = model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)] * currel_errorpose_;
    // // std::cout << "currel_errorpose_: " << currel_errorpose_ << std::endl;
    // // 3.2 使用自旋锁保护共享资源
    // while (spin_lock.test_and_set(std::memory_order_acquire))
    //   ;
    // // 3.3 计算力矩误差，并保存在currel_errorpose_rb_中，方便update函数中调用
    // currel_errorpose_rb_.writeFromNonRT(currel_errorpose_);
    // // 3.4 释放自旋锁
    // spin_lock.clear(std::memory_order_release);
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      targetWrenchCB(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    // ------------1.赋值给target_wrench_--------------
    target_wrench_.linear()(0) = msg->wrench.force.x;
    target_wrench_.linear()(1) = msg->wrench.force.y;
    target_wrench_.linear()(2) = msg->wrench.force.z;
    target_wrench_.angular()(0) = msg->wrench.torque.x;
    target_wrench_.angular()(1) = msg->wrench.torque.y;
    target_wrench_.angular()(2) = msg->wrench.torque.z;

    // target_wrench_.linear()(0) = 0;
    // target_wrench_.linear()(1) = 0;
    // target_wrench_.linear()(2) = msg->wrench.force.z;
    // target_wrench_.angular()(0) = 0;
    // target_wrench_.angular()(1) = 0;
    // target_wrench_.angular()(2) = 0;

    // ------------2.从实时数据缓冲区读取机械臂当前状态信息--------------
    model_temp_data_ = *(pino_rstate_rb_.readFromRT());
    // ------------3.接收参考力矩--------------
    // 3.1 使用自旋锁保护共享资源
    while (spin_lock.test_and_set(std::memory_order_acquire))
      ;
    // 3.2 转换wrench的参考坐标系为right/left_base_link
    target_wrench_ = model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].act(target_wrench_);
    // std::cout << "target_wrench: " << target_wrench_ << std::endl;
    // 3.3 释放自旋锁
    spin_lock.clear(std::memory_order_release);
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      dynamicReconfigureCallback(selfdefined_trajectory_controller::selfdefinedControllerConfig
                                     &config,
                                 uint32_t level)
  {
    Vector6D tmp_M, tmp_D, tmp_K;
    for (size_t i = 0; i < 3; i++)
    {
      tmp_M[i] = config.F_M_para;
      tmp_D[i] = config.F_D_para;
      tmp_K[i] = config.F_K_para;
      tmp_M[i + 3] = config.T_M_para;
      tmp_D[i + 3] = config.T_D_para;
      tmp_K[i + 3] = config.T_K_para;
    }
    this->AD_kp = config.AD_kp;
    this->AD_ki = config.AD_ki;
    this->AD_kd = config.AD_kd;
    this->M_ = tmp_M.asDiagonal();
    this->D_ = tmp_D.asDiagonal();
    this->K_ = tmp_K.asDiagonal();
    this->Coor_para = config.Coor_para;
  }

  template <class SegmentImpl, class HardwareInterface>
  bool SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      updateTrajectoryCommand(const JointTrajectoryConstPtr &msg,
                              RealtimeGoalHandlePtr gh,
                              std::string *error_string)
  {
    typedef joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> Options;
    Options options;
    options.error_string = error_string;
    std::string error_string_tmp;

    // ------------1.预检查条件：检查控制器是否正在运行--------------
    if (!this->isRunning())
    {
      error_string_tmp = "Can't accept new commands. Controller is not running.";
      ROS_ERROR_STREAM_NAMED(this->name_, error_string_tmp);
      options.setErrorString(error_string_tmp);
      return false;
    }
    // ------------2.检查传入的轨迹消息指针是否为空--------------
    if (!msg)
    {
      error_string_tmp = "Received null-pointer trajectory message, skipping.";
      ROS_WARN_STREAM_NAMED(this->name_, error_string_tmp);
      options.setErrorString(error_string_tmp);
      return false;
    }

    // ------------3.获取时间数据--------------
    // 3.1 读取时间数据
    typename Base::TimeData *time_data = this->time_data_.readFromRT();
    // 3.2 计算下一次更新的时间
    const ros::Time next_update_time = time_data->time + time_data->period;
    // 3.3 计算下一次更新的正常运行时间
    ros::Time next_update_uptime = time_data->uptime + time_data->period;

    // ------------4.检查轨迹是否为空--------------
    if (msg->points.empty())
    {
      this->setHoldPosition(time_data->uptime, gh);
      ROS_DEBUG_NAMED(this->name_, "Empty trajectory command, stopping.");
      return true;
    }

    // ------------5.设置轨迹初始化选项--------------
    // 5.1 轨迹初始化选项声明
    TrajectoryPtr curr_traj_ptr;
    this->curr_trajectory_box_.get(curr_traj_ptr);
    // 5.2 设置选项
    options.other_time_base = &next_update_uptime;
    options.current_trajectory = curr_traj_ptr.get();
    options.joint_names = &this->joint_names_;
    options.angle_wraparound = &this->angle_wraparound_;
    options.rt_goal_handle = gh;
    options.default_tolerances = &this->default_tolerances_;
    options.allow_partial_joints_goal = this->allow_partial_joints_goal_;

    // ------------6.更新当前正在执行的轨迹--------------
    //////////////////实时更新轨迹/////////////////////
    try
    {
      TrajectoryPtr traj_ptr(new Trajectory);
      *traj_ptr = joint_trajectory_controller::
          initJointTrajectory<Trajectory>(*msg, next_update_time, options);
      if (!traj_ptr->empty())
      { // 将新的轨迹设置为当前轨迹
        this->curr_trajectory_box_.set(traj_ptr);
      }
      else
      {
        return false;
      }
    }
    /////////////////////////////////////////////////
    // 捕获异常
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM_NAMED(this->name_, ex.what());
      options.setErrorString(ex.what());
      return false;
    }
    catch (...)
    {
      error_string_tmp = "Unexpected exception caught when initializing "
                         "trajectory from ROS message data.";
      ROS_ERROR_STREAM_NAMED(this->name_, error_string_tmp);
      options.setErrorString(error_string_tmp);
      return false;
    }
    return true;
  }

  template <class SegmentImpl, class HardwareInterface>
  inline void
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      trajectoryCommandCB(const JointTrajectoryConstPtr &msg)
  {
    // 调用 updateTrajectoryCommand 函数来更新轨迹命令
    const bool update_ok = updateTrajectoryCommand(msg, RealtimeGoalHandlePtr());
    // 如果轨迹更新成功，则终止当前的活动目标
    if (update_ok)
    {
      preemptActiveGoal();
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  inline void
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      dualtrajectoryCommandCB(const JointTrajectoryConstPtr &msg)
  {
    trajectory_msgs::JointTrajectoryConstPtr single_msg;
    // 确定当前控制器是左/右机械臂------------------
    if (this->robot_name_ == "left_robot")
      single_msg = processTrajectory(msg, false);
    else if (this->robot_name_ == "right_robot")
      single_msg = processTrajectory(msg, true);
    // 确定当前控制器是左/右机械臂------------------

    // 调用 updateTrajectoryCommand 函数来更新轨迹命令
    const bool update_ok = updateTrajectoryCommand(single_msg, RealtimeGoalHandlePtr());
    // 如果轨迹更新成功，则终止当前的活动目标
    if (update_ok)
    {
      preemptActiveGoal();
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  inline void
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::preemptActiveGoal()
  {
    RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
    // 检查当前活动目标
    if (current_active_goal)
    {
      // 重置和取消活动目标
      this->rt_active_goal_.reset();
      current_active_goal->gh_.setCanceled();
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<
      SegmentImpl, HardwareInterface>::goalCB(GoalHandle gh)
  {
    ROS_DEBUG_STREAM_NAMED(this->name_, "Received new action goal");

    // ------------1.处理控制器未运行的情况--------------
    // 检查控制器是否在运行
    if (!this->isRunning())
    {
      ROS_ERROR_NAMED(this->name_,
                      "Can't accept new action goals. Controller is not running.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL; // TODO: Add better error status to msg?
      gh.setRejected(result);
      return;
    }

    // ------------2.处理部分目标不被允许的情况--------------
    // 部分目标（部分关节目标）是否被允许
    if (!this->allow_partial_joints_goal_)
    {
      if (gh.getGoal()->trajectory.joint_names.size() !=
          this->joint_names_.size())
      {
        ROS_ERROR_NAMED(this->name_,
                        "Joints on incoming goal don't match the controller joints.");
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        gh.setRejected(result);
        return;
      }
    }

    // ------------3.处理关节名称不匹配的情况--------------
    using joint_trajectory_controller::internal::mapping;
    std::vector<unsigned int> mapping_vector =
        mapping(gh.getGoal()->trajectory.joint_names, this->joint_names_);
    // 目标关节名称与控制器不匹配
    if (mapping_vector.empty())
    {
      ROS_ERROR_NAMED(this->name_,
                      "Joints on incoming goal don't match the controller joints.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(result);
      return;
    }

    // ------------4.尝试更新新的轨迹目标并处理相关的反馈--------------
    RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
    std::string error_string;
    // 尝试更新轨迹（update_ok更新是否成功）
    const bool update_ok = updateTrajectoryCommand(
        joint_trajectory_controller::internal::share_member(
            gh.getGoal(), gh.getGoal()->trajectory),
        rt_goal, &error_string);
    // 设置反馈中的关节名称
    rt_goal->preallocated_feedback_->joint_names = this->joint_names_;

    // ------------5.根据update_ok决定是否接受或拒绝新的目标--------------
    if (update_ok)
    {
      // 接受新目标
      preemptActiveGoal();
      gh.setAccepted();
      this->rt_active_goal_ = rt_goal;

      // 设置目标状态检查定时器
      this->goal_handle_timer_ = this->controller_nh_.createTimer(
          this->action_monitor_period_, &RealtimeGoalHandle::runNonRealtime, rt_goal);
      this->goal_handle_timer_.start();
    }
    else
    {
      // 处理更新失败的情况
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      result.error_string = error_string;
      gh.setRejected(result);
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<
      SegmentImpl, HardwareInterface>::cancelCB(GoalHandle gh)
  {
    // 1.创建当前活动目标的实时句柄
    RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);

    // 2.检查取消请求是否针对当前活动目标
    if (current_active_goal && current_active_goal->gh_ == gh)
    {
      // 2.1 重置当前目标
      this->rt_active_goal_.reset();
      // 2.2 获取控制器运行时间
      const ros::Time uptime = this->time_data_.readFromRT()->uptime;
      // 2.3 进入保持当前位置模式
      this->setHoldPosition(uptime);
      ROS_DEBUG_NAMED(this->name_, "Canceling active action goal because cancel "
                                   "callback recieved from actionlib.");
      // 2.4 将当前目标标记为已取消
      current_active_goal->gh_.setCanceled();
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  bool SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      queryStateService(control_msgs::QueryTrajectoryState::Request &req,
                        control_msgs::QueryTrajectoryState::Response &resp)
  {
    // 1.检查控制器是否正在运行
    if (!this->isRunning())
    {
      ROS_ERROR_NAMED(this->name_,
                      "Can't sample trajectory. Controller is not running.");
      return false;
    }

    // 2.将请求时间转换为内部单调表示
    typename Base::TimeData *time_data = this->time_data_.readFromRT();
    const ros::Duration time_offset = req.time - time_data->time;
    const ros::Time sample_time = time_data->uptime + time_offset;

    // 3.获取当前轨迹
    TrajectoryPtr curr_traj_ptr;
    this->curr_trajectory_box_.get(curr_traj_ptr);
    Trajectory &curr_traj = *curr_traj_ptr;

    // 4.初始化响应点状态
    typename Segment::State response_point =
        typename Segment::State(this->joint_names_.size());

    // 5.采样轨迹状态
    for (unsigned int i = 0; i < this->getNumberOfJoints(); ++i)
    {
      typename Segment::State state;
      typename TrajectoryPerJoint::const_iterator segment_it =
          sample(curr_traj[i], sample_time.toSec(), state);
      if (curr_traj[i].end() == segment_it)
      {
        ROS_ERROR_STREAM_NAMED(
            this->name_, "Requested sample time precedes trajectory start time.");
        return false;
      }

      response_point.position[i] = state.position[0];
      response_point.velocity[i] = state.velocity[0];
      response_point.acceleration[i] = state.acceleration[0];
    }

    // 6.填充响应消息
    resp.name = this->joint_names_;
    resp.position = response_point.position;
    resp.velocity = response_point.velocity;
    resp.acceleration = response_point.acceleration;

    return true;
  }

  template <class SegmentImpl, class HardwareInterface>
  inline void
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      admittance_control(const ros::Duration &period)
  {
    // -----------------------------导纳控制器------------------------------------------
    // ------------6.读取力矩误差---------------
    error_wrench_ = *(error_wrench_rb_.readFromRT());
    // std::cout << "error_wrench : " << error_wrench_ << std::endl;

    // ////////////////////////////////////////////////////////////
    // // 自适应阻抗
    // Adaptive_Damp_Z(period);
    // Vector6D tmp_dD;
    // for (size_t i = 0; i < tmp_dD.size(); i++)
    //   tmp_dD[i] = dD_value[i];
    // Eigen::Matrix<double, 6, 6> d_D;
    // d_D = tmp_dD.asDiagonal();
    // ////////////////////////////////////////////////////////////

    // ------------7.导纳控制器算法--------------
    // (因为使用的是上个周期的关节状态，故x_e与dx_e要加 desired_***_*period.toSec())
    // 7.1 当前末端位置和期望末端位置的差距(仅由外力造成,不考虑轨迹跟踪)
    // pinocchio::log6(curr_pose_.actInv(desired_pose_)）: desired_pose_在curr_pose_下的表示
    // curr_pose_.act(-pinocchio::log6(curr_pose_.actInv(desired_pose_))): curr_pose_ - desired_pose_'
    x_e_ = curr_pose_.act(-pinocchio::log6(curr_pose_.actInv(desired_pose_))) +
           desired_vel_ * period.toSec(); // pinocchio::Motion (最后参考到curr_pose_所在的base_link坐标系)

    // std::cout << "---x_e_---" << x_e_.toVector().transpose() << std::endl;
    // 7.2 当前末端速度和期望末端速度的差距（仅由外力造成,不考虑轨迹跟踪）
    dx_e_ = pinocchio::Motion(curr_vel_ - desired_vel_ +
                              desired_acc_ * period.toSec());
    // 7.3 导纳控制率（仅由外力造成,不考虑轨迹跟踪）,计算末端加速度
    ddx_e_ = pinocchio::Motion(M_.inverse() *
                               (error_wrench_ - K_ * x_e_.toVector() - D_ * dx_e_.toVector()));

    // ------------8.速度/加速度缩放--------------
    // 8.1 加速度缩放,避免过大
    double traAccNorm = ddx_e_.linear().norm(); // 平移加速度
    if (traAccNorm > traAccLimit_)
    {
      ddx_e_ = ddx_e_ * traAccLimit_ / traAccNorm;
      ROS_WARN_STREAM("traAccNorm: " << traAccNorm << " is larger than " << traAccLimit_);
    }
    double rotAccNorm = ddx_e_.angular().norm(); // 旋转加速度
    if (rotAccNorm > rotAccLimit_)
    {
      ddx_e_ = ddx_e_ * rotAccLimit_ / rotAccNorm;
      ROS_WARN_STREAM("rotAccNorm: " << rotAccNorm << " is larger than " << rotAccLimit_);
    }
    // 8.2 计算末端期望速度------------------（最终输出一个末端期望速度期望）
    desired_vel_ = curr_vel_ + (desired_acc_ + ddx_e_) * period.toSec();
    // 8.3 速度缩放,避免速度过大
    double traVelNorm = desired_vel_.linear().norm(); // *平移速度
    if (traVelNorm > traVelLimit_)
    {
      desired_vel_ = desired_vel_ * traVelLimit_ / traVelNorm;
      ROS_WARN_STREAM("traVelNorm: " << traVelNorm << " is larger than " << traVelLimit_);
    }
    double rotVelNorm = desired_vel_.angular().norm(); // *旋转速度
    if (rotVelNorm > rotVelLimit_)
    {
      desired_vel_ = desired_vel_ * rotVelLimit_ / rotVelNorm;
      ROS_WARN_STREAM("rotVelNorm: " << rotVelNorm << " is larger than " << rotVelLimit_);
    }

    // ------------9.计算机器人期望状态--------------(关节空间q积分)
    // 8.1 计算当前状态下的雅可比矩阵
    J_.setZero();
    pinocchio::computeFrameJacobian(robot_model_, model_curr_data_,
                                    pino_curr_pos_, compliance_frameId_,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J_);
    // 8.2 计算期望关节速度（末端笛卡尔坐标——>关节空间）
    pino_desired_vel_ = J_.completeOrthogonalDecomposition().pseudoInverse() *
                        desired_vel_.toVector();
    // 8.3 计算期望关节角（基于pino_curr_pos_与上述求得pino_desired_vel_）
    pino_desired_pos_ = pinocchio::integrate(robot_model_, pino_curr_pos_,
                                             pino_desired_vel_ * period.toSec());
    // 8.4 将期望状态赋给desired_state_作为输出
    for (size_t i = 0; i < this->joints_.size(); ++i)
    {
      this->desired_state_.position[i] = pino_desired_pos_(i);
      this->desired_state_.velocity[i] = pino_desired_vel_(i);
      // pino_desired_acc_未赋值，保持zero
      this->desired_state_.acceleration[i] = pino_desired_acc_(i);
    }
    // ---------------------------------end-------------------------------------------
    // ------------得到desired_state_，然后会发送到控制硬件
  }

  template <class SegmentImpl, class HardwareInterface>
  inline double*
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      Adaptive_Damp_Z(const ros::Duration &period)
  {
    // 1.获取tool坐标系下的error_wrench
    pinocchio::Force error_current;
    error_current = error_wrench_;
    error_current = model_temp_data_.oMf[compliance_frameId_].actInv(error_current);

    // 赋值
    double Damp_proportion[6], Damp_integral[6], Damp_difference[6];

    for(int i=0; i<6; i++)
    { 
      if(i < 3){
        error_zforce[i][0] = error_zforce[i][1];
        error_zforce[i][1] = error_zforce[i][2];
        error_zforce[i][2] = error_current.linear()(i);
      }
      else
      {
        error_zforce[i][0] = error_zforce[i][1];
        error_zforce[i][1] = error_zforce[i][2];
        error_zforce[i][2] = error_current.angular()(i-3);
      }

      // 增量式pid
      Damp_proportion[i] = ((error_zforce[i][2] - error_zforce[i][1]) * AD_kp);
      Damp_integral[i] = (error_zforce[i][2] * AD_ki * 0.01);
      Damp_difference[i] = ((error_zforce[i][2] + error_zforce[i][0] - 2 * error_zforce[i][1]) * AD_kd);

      // 积分限幅（抗积分饱和）
      if (fabs(Damp_integral[i]) >= 9 * 80 / 10)
        if (Damp_integral[i] > 0)
          Damp_integral[i] = 9 * 80 / 10;
        else
          Damp_integral[i] = -9 * 80 / 10;

      dD_value[i] += (Damp_proportion[i] + Damp_integral[i] + Damp_difference[i]);

      // 电机限幅
      if (abs(dD_value[i]) >= 0.95 * 80)
      {
        if (dD_value[i] > 0)
          dD_value[i] = 0.95 * 80;
        else
          dD_value[i] = -0.95 * 80;
      }

    }
    // if(this->robot_name_ == "left_robot")
    //   std::cout << "dD_value: " << dD_value[0] <<" " <<  dD_value[1] << " "<< dD_value[2] << " "
    //                             << dD_value[3] <<" " <<  dD_value[4] << " "<< dD_value[5]
    //                             << std::endl;
    return dD_value;
  }

  template <class SegmentImpl, class HardwareInterface>
  inline std::string
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      getParentNamespace(const ros::NodeHandle &nh, int level)
  {
    std::string ns = nh.getNamespace(); // 获取完整命名空间
    if (ns.empty() || ns[0] != '/')
    {
      ns = "/" + ns; // 确保以 '/' 开头
    }

    // 按 '/' 分割命名空间
    std::vector<std::string> tokens;
    std::stringstream ss(ns);
    std::string token;
    while (std::getline(ss, token, '/'))
    {
      if (!token.empty())
      {
        tokens.push_back(token);
      }
    }

    // 检查层级是否有效
    if (level <= 0 || level > static_cast<int>(tokens.size()))
    {
      return ""; // 如果层级无效，返回空字符串
    }

    return tokens[level - 1]; // 返回指定层级的命名空间
  }

  template <class SegmentImpl, class HardwareInterface>
  inline trajectory_msgs::JointTrajectoryPtr
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      processTrajectory(const trajectory_msgs::JointTrajectoryConstPtr &msg, bool use_last_six)
  {
    // 创建一个新的 JointTrajectory 消息
    trajectory_msgs::JointTrajectoryPtr single_msg(new trajectory_msgs::JointTrajectory);
    // 确定需要提取的起始和结束索引
    const size_t total_joints = msg->joint_names.size();
    const size_t start_index = use_last_six ? total_joints - 6 : 0; // 前6个 or 后6个
    const size_t end_index = start_index + 6;
    // 拷贝头部信息
    single_msg->header = msg->header;
    // 拷贝需要的关节名称
    single_msg->joint_names.insert(single_msg->joint_names.end(),
                                   msg->joint_names.begin() + start_index,
                                   msg->joint_names.begin() + end_index);
    // 遍历 msg 的每个轨迹点
    for (const auto &point : msg->points)
    {
      // 创建一个新的 TrajectoryPoint
      trajectory_msgs::JointTrajectoryPoint single_point;
      // 提取需要的关节值
      single_point.positions.insert(single_point.positions.end(),
                                    point.positions.begin() + start_index,
                                    point.positions.begin() + end_index);
      if (!point.velocities.empty())
      {
        single_point.velocities.insert(single_point.velocities.end(),
                                       point.velocities.begin() + start_index,
                                       point.velocities.begin() + end_index);
      }
      if (!point.accelerations.empty())
      {
        single_point.accelerations.insert(single_point.accelerations.end(),
                                          point.accelerations.begin() + start_index,
                                          point.accelerations.begin() + end_index);
      }
      if (!point.effort.empty())
      {
        single_point.effort.insert(single_point.effort.end(),
                                   point.effort.begin() + start_index,
                                   point.effort.begin() + end_index);
      }
      // 保留时间
      single_point.time_from_start = point.time_from_start;
      // 添加到 single_msg 的轨迹点
      single_msg->points.push_back(single_point);
    }
    // 返回处理后的轨迹消息
    return single_msg;
  }

} // namespace selfdefined_controllers
