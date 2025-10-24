#include "manipulator/robot_arm.h"

namespace Robot_arm
{
    /*------------------------单臂区--------------------------------*/
    Single_arm::Single_arm(ros::NodeHandle &nh,
                           moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                           moveit_visual_tools::MoveItVisualToolsPtr vtptr)
    {
        // 赋值
        this->nh_ = nh;
        this->move_group_ptr_ = mgptr;
        visual_tools_ptr_ = vtptr;

        // 加载机器人模型
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robotmodel_ptr_ = robot_model_loader.getModel();

        // 获取关节组名称
        std::string robotgroup_name = move_group_ptr_->getName();
        joint_model_group = move_group_ptr_->getCurrentState()->getJointModelGroup(robotgroup_name);
        moveit::core::RobotStatePtr current_state = move_group_ptr_->getCurrentState();
        // 获得关节角名称
        std::vector<std::string> temp_joint_names_ = move_group_ptr_->getJoints();
        for (size_t i = 0; i < temp_joint_names_.size(); i++)
        {
            size_t temp_i = i % 9;
            if ((temp_i == 0) || temp_i > 6)
                continue;
            else
                joint_names_.push_back(temp_joint_names_[i]);
        }

        // 记录初始关节
        current_state->copyJointGroupPositions(joint_model_group, Initial_joint_group);

        // 一些基本话题订阅与发布
        // Pliz库相关话题
        seq_client_ = nh_.serviceClient<moveit_msgs::GetMotionSequence>("/plan_sequence_path");
        seq_client_.waitForExistence();
        circ_client_ = nh_.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");
        circ_client_.waitForExistence();
        // moveit！相关
        planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

        // 加载已保存的关节角与末端位姿
        // std::vector<std::string> station_names{"left_robot_JP", "right_robot_JP"};
        std::vector<std::string> station_names{"left_grindrobot_JP", "right_grindrobot_JP"};
        for (std::string &station_name : station_names)
        {
            load_joint_pose(station_name);
        }

        ROS_INFO_NAMED("Single_arm", "Available Planning Groups:");
        std::copy(move_group_ptr_->getJointModelGroupNames().begin(),
                  move_group_ptr_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
        std::cout << std::endl;

        // 检查环境话题是否有效
        while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
        {
            ros::WallDuration sleep_t(0.5);
            sleep_t.sleep();
            ROS_INFO_NAMED("Single_arm", "Wait the topic of %s response ...", "planning_scene");
        }

        ROS_INFO_NAMED("Single_arm", "Single_arm: All is in order!");
    }

    Single_arm::~Single_arm()
    {
        ROS_INFO_NAMED("Single_arm", "This Class is to free.....");
    }

    /*------------------运动规划函数------------------------------*/
    bool Single_arm::move_ptp(std::string name, JPtype jptype, bool execute_flag)
    {

        std::vector<double> joints;
        geometry_msgs::Pose abs_pose;

        move_group_ptr_->setStartStateToCurrentState();

        move_group_ptr_->setPlannerId("PTP");

        switch (jptype)
        {
        case JOINT:
            if(!get_joint(name,joints))
                return false;
            move_group_ptr_->setJointValueTarget(joints);
            break;

        case POINT:
            if(!get_joint(name + "_inverse",joints))
                return false;
            if(!get_abs_pose(name,abs_pose))
                return false;
            robot_state::RobotState rstate(robotmodel_ptr_);
            rstate.setToDefaultValues();
            rstate.setVariablePositions(joint_names_, joints);
            robotmodel_ptr_->getJointModelGroup(move_group_ptr_->getName());

            rstate.setFromIK(robotmodel_ptr_->getJointModelGroup(move_group_ptr_->getName()),
                             abs_pose, move_group_ptr_->getEndEffectorLink());

            move_group_ptr_->setJointValueTarget(rstate);
            break;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_ptr_->plan(plan) !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR_STREAM("Default pose can not be reached!");
            return false;
        }

        // 规划路径赋值
        my_plan_ = plan;
        // 将规划出来的路径通过rviz可视化
        visual_tools_ptr_->publishTrajectoryLine(my_plan_.trajectory_, joint_model_group);
        visual_tools_ptr_->trigger();

        if (execute_flag)
        {
            ROS_INFO_NAMED("Single_arm", "Planning and execution of the movement succeeded!");
            move_group_ptr_->execute(my_plan_);
        }
        else
        {
            ROS_INFO_NAMED("Single_arm", "Only planning of the movement realizes!");
        }
        visual_tools_ptr_->deleteAllMarkers();
        return true;
    }

    bool Single_arm::move_line(Movetowards towards, double dis, double speed_factor, bool execute_flag)
    {
        move_group_ptr_->setPlannerId("LIN");
        moveit::planning_interface::MoveGroupInterface::Plan Lineplan;
        geometry_msgs::PoseStamped target_pose = move_group_ptr_->getCurrentPose();
        switch (towards)
        {
        case UP:
            target_pose.pose.position.z += dis;
            break;
        case DOWN:
            target_pose.pose.position.z -= dis;
            break;
        case LEFT:
            target_pose.pose.position.x -= dis;
            break;
        case RIGHT:
            target_pose.pose.position.x += dis;
            break;
        case FORWARD:
            target_pose.pose.position.y += dis;
            break;
        case BACKWARD:
            target_pose.pose.position.y -= dis;
            break;
        }
        move_group_ptr_->setPoseTarget(target_pose);
        move_group_ptr_->setMaxVelocityScalingFactor(speed_factor);
        if (move_group_ptr_->plan(Lineplan) !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO_NAMED("Pliz_move", "Plan has failed......");
            return false;
        }
        // 规划路径赋值
        my_plan_ = Lineplan;
        // 将规划出来的路径通过rviz可视化
        visual_tools_ptr_->publishTrajectoryLine(my_plan_.trajectory_, joint_model_group);
        visual_tools_ptr_->trigger();
        if (execute_flag)
        {
            ROS_INFO_NAMED("Single_arm", "Planning and execution of the movement succeeded!");
            move_group_ptr_->execute(my_plan_);
        }
        else
        {
            ROS_INFO_NAMED("Single_arm", "Only planning of the movement realizes!");
        }
        move_group_ptr_->setMaxVelocityScalingFactor(NORMAL_SPEED);
        visual_tools_ptr_->deleteAllMarkers();
        return true;
    }

    bool Single_arm::move_circ(CirType type, std::string auxiliary_point,
                               std::string goal_point, bool execute_flag)
    {
        geometry_msgs::Pose abs_pose;
        if(!get_abs_pose(auxiliary_point,abs_pose))
            return false;
        move_group_ptr_->setPlannerId("CIRC");

        moveit_msgs::MotionPlanRequest req;

        move_group_ptr_->constructMotionPlanRequest(req);
        req.group_name = move_group_ptr_->getName();
        req.goal_constraints.resize(1);
        req.path_constraints.position_constraints.resize(1);
        req.max_velocity_scaling_factor = CLOSE_SPEED;
        req.max_acceleration_scaling_factor = CLOSE_ACC;
        req.planner_id = "CIRC";
        if (type == CENTER)
            req.path_constraints.name = "center";
        else if (type == INTERIM)
            req.path_constraints.name = "interim";

        moveit_msgs::PositionConstraint posConstraint;
        posConstraint.link_name = move_group_ptr_->getEndEffectorLink();

        posConstraint.constraint_region.primitive_poses.push_back(abs_pose);
        req.path_constraints.position_constraints[0] = posConstraint;

        moveit_msgs::Constraints pose_goal;
        geometry_msgs::Pose next_pose;
        get_constraint(goal_point, pose_goal, next_pose, POINT);
        req.goal_constraints[0] = pose_goal;

        moveit_msgs::GetMotionPlan srvreq;
        srvreq.request.motion_plan_request = req;
        srvreq.response.motion_plan_response.error_code.val = 0;
        circ_client_.call(srvreq);

        visual_tools_ptr_->publishTrajectoryLine(srvreq.response.motion_plan_response.trajectory, joint_model_group);
        visual_tools_ptr_->trigger();
        // 是否进行执行
        if (execute_flag)
        {
            ROS_INFO_NAMED("Single_arm", "Planning and execution of the movement succeeded!");
            move_group_ptr_->execute(srvreq.response.motion_plan_response.trajectory);
        }
        else
        {
            ROS_INFO_NAMED("Single_arm", "Only planning of the movement realizes!");
        }
        move_group_ptr_->setPlannerId("PTP");
        visual_tools_ptr_->deleteAllMarkers();
        return true;
    }

    void Single_arm::move_seq(std::vector<std::string> &V_name, std::vector<double> &V_radius,
                              std::vector<double> &V_speed, std::vector<double> &V_acc,
                              std::vector<std::string> &V_planner,
                              bool execute_flag)
    {
        moveit_msgs::GetMotionSequence srvseq;
        srvseq.request.request =
            construct_seq_req(V_name, V_radius, V_speed, V_acc, V_planner);
        srvseq.response.response.error_code.val = 0;

        int replan_num = 5;
        int replan_count = 0;

        while (srvseq.response.response.error_code.val != 1 &&
               replan_count < replan_num)
        {
            seq_client_.call(srvseq);
            if (srvseq.response.response.error_code.val != 1)
            {
                // 将混合半径设为0，再次尝试规划
                ROS_WARN_STREAM(
                    "BLENDING FAILED! Try setting blend_radius to 0 and plan again...");
                std::vector<double>(V_radius.size(), 0).swap(V_radius);
                srvseq.request.request =
                    construct_seq_req(V_name, V_radius, V_speed, V_acc, V_planner);
            }
            ++replan_count;
        }

        if (srvseq.response.response.error_code.val != 1)
        {
            ROS_ERROR_STREAM("plan sequence_goal failed.ERROR CODE is"
                             << srvseq.response.response.error_code.val);
        }
        visual_tools_ptr_->publishTrajectoryLine(srvseq.response.response.planned_trajectories[0], joint_model_group);
        visual_tools_ptr_->trigger();

        my_plan_accessed.trajectory_ = srvseq.response.response.planned_trajectories[0];

        // 是否进行执行
        if (execute_flag)
        {
            ROS_INFO_NAMED("Single_arm", "Planning and execution of the movement succeeded!");
            move_group_ptr_->execute(srvseq.response.response.planned_trajectories[0]);
        }
        else
        {
            ROS_INFO_NAMED("Single_arm", "Only planning of the movement realizes!");
        }

        visual_tools_ptr_->deleteAllMarkers();
    }

    void Single_arm::pliz_seqmove(void)
    {
        std::vector<std::string> V_name1{
            "circle1_inference_inverse",
            "circle2_inference_inverse",
            "circle3_inference_inverse",
            "circle0_inference_inverse",
            "circle2_inference_inverse",
            "circle3_inference_inverse",
            "circle0_inference_inverse",
            "circle1_inference_inverse",
            "circle3_inference_inverse",
            "circle0_inference_inverse",
            "circle1_inference_inverse",
            "circle2_inference_inverse",
            "circle0_inference_inverse",
            "circle1_inference_inverse",
            "circle2_inference_inverse",
            "circle3_inference_inverse",
            "bottom_pose_inference_inverse",
        };
        std::vector<double> V_radius1{
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
            0.08,
        };
        std::vector<double> V_speed1{
            NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
            NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
            NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
            NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
            NORMAL_SPEED};
        std::vector<double> V_acc1{
            NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
            NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
            NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
            NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
            NORMAL_ACC};
        std::vector<std::string> V_planner1{
            "PTP", "PTP", "PTP", "PTP",
            "PTP", "PTP", "PTP", "PTP",
            "PTP", "PTP", "PTP", "PTP",
            "PTP", "PTP", "PTP", "PTP",
            "PTP"};
        move_seq(V_name1, V_radius1, V_speed1, V_acc1, V_planner1);
    }
    void Single_arm::pliz_seqmove_plus(int target_order[], int array_name, bool execute_flag)
    {
        std::vector<std::string> V_name1;
        std::vector<double> V_radius1;
        std::vector<double> V_speed1;
        std::vector<double> V_acc1;
        std::vector<std::string> V_planner1;

        for (size_t i = 0; i < array_name; i++)
        {
            std::string target_name = "right_circle" + std::to_string(target_order[i]) + "_inverse";
            V_name1.push_back(target_name);
            V_radius1.push_back(0.071);
            V_speed1.push_back(NORMAL_SPEED);
            V_acc1.push_back(NORMAL_ACC);
            V_planner1.push_back("PTP");
        }

        // 回到碗底
        // V_name1.push_back("right_bottom_inverse");
        // V_radius1.push_back(0.071);
        // V_speed1.push_back(NORMAL_SPEED);
        // V_acc1.push_back(NORMAL_ACC);
        // V_planner1.push_back("PTP");

        move_seq(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, execute_flag);
    }
    bool Single_arm::move_targetPose(geometry_msgs::Pose &target_pose, bool execute_flag, bool echo, int retry)
    {
        if(echo)ROS_INFO_NAMED("Single_arm", "Will reach  the target pose...");

        move_group_ptr_->setStartStateToCurrentState();
        move_group_ptr_->setPoseTarget(target_pose);

        /*------------------规划和执行运动--------------------*/
        // 规划成功标志
        bool success;
        success = (move_group_ptr_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
        my_plan_accessed = my_plan_;
        // 将规划出来的路径通过rviz可视化
        visual_tools_ptr_->publishTrajectoryLine(my_plan_.trajectory_, joint_model_group);
        visual_tools_ptr_->trigger();

        /*----------------------------------------*/
        if (success)
        {
            if (execute_flag)
            {
                moveit::core::MoveItErrorCode execute_result = move_group_ptr_->execute(my_plan_);
                // 判断执行是否成功
                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    if(echo)ROS_INFO_NAMED("Single_arm", "Planning and execution succeeded!");
                }
                else
                {
                    ROS_ERROR_NAMED("Single_arm", "Execution failed! Error code: %d", execute_result.val);
                    visual_tools_ptr_->deleteAllMarkers();
                    return false; // 执行失败返回false
                }
            }
            else
            {
                if(echo)ROS_INFO_NAMED("Single_arm", "Only planning of the movement realizes!");
            }
            visual_tools_ptr_->deleteAllMarkers();
            return true;
        }
        else
        {
            ROS_ERROR_NAMED("Single_arm", "Planning of the movement failed! (%d)",retry);
            visual_tools_ptr_->deleteAllMarkers();
            if(retry--){
                ROS_WARN_NAMED("Single_arm", "Retry planning the movement!");
                return move_targetPose(target_pose,execute_flag,echo,retry);
            }
            return false;
        }
    }

    bool Single_arm::move_targetJoints(std::vector<double> &joint_pointions, bool execute_flag, bool echo, int retry)
    {
        if(echo)ROS_INFO_NAMED("Single_arm", "Will reach  the target joints...");

        move_group_ptr_->setStartStateToCurrentState();
        move_group_ptr_->setJointValueTarget(joint_pointions);
        /*------------------规划和执行运动--------------------*/
        // 规划成功标志
        bool success;
        success = (move_group_ptr_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
        // 将规划出来的路径通过rviz可视化
        visual_tools_ptr_->publishTrajectoryLine(my_plan_.trajectory_, joint_model_group);
        visual_tools_ptr_->trigger();

        /*----------------------------------------*/
        if (success)
        {
            if (execute_flag)
            {
                moveit::core::MoveItErrorCode execute_result = move_group_ptr_->execute(my_plan_);
                // 判断执行是否成功
                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    if(echo)ROS_INFO_NAMED("Single_arm", "Planning and execution succeeded!");
                }
                else
                {
                    ROS_ERROR_NAMED("Single_arm", "Execution failed! Error code: %d", execute_result.val);
                    visual_tools_ptr_->deleteAllMarkers();
                    return false; // 执行失败返回false
                }
            }
            else
            {
                if(echo)ROS_INFO_NAMED("Single_arm", "Only planning of the movement realizes!");
            }
            visual_tools_ptr_->deleteAllMarkers();
            return true;
        }
        else
        {
            ROS_ERROR_NAMED("Single_arm", "Planning of the movement failed! (%d)",retry);
            visual_tools_ptr_->deleteAllMarkers();
            if(retry--){
                ROS_WARN_NAMED("Single_arm", "Retry planning the movement!");
                return move_targetJoints(joint_pointions,execute_flag,echo,retry);
            }
            return false;
        }
    }
    /*------------------运动规划函数------------------------------*/

    /*------------------获得相关信息的函数------------------------------*/
    double Single_arm::get_blend_radius(geometry_msgs::Pose &curr_pose,
                                        geometry_msgs::Pose &next_pose,
                                        geometry_msgs::Pose &n_next_pose)
    {
        // 若curr_pose和n_next_pose重合,将融合半径置0
        if (pow(n_next_pose.position.x - curr_pose.position.x, 2) +
                pow(n_next_pose.position.y - curr_pose.position.y, 2) +
                pow(n_next_pose.position.z - curr_pose.position.z, 2) <
            0.0025)
            return 0.0;
        double radius1 = sqrt(pow(next_pose.position.x - curr_pose.position.x, 2) +
                              pow(next_pose.position.y - curr_pose.position.y, 2) +
                              pow(next_pose.position.z - curr_pose.position.z, 2));
        double radius2 = sqrt(pow(n_next_pose.position.x - next_pose.position.x, 2) +
                              pow(n_next_pose.position.y - next_pose.position.y, 2) +
                              pow(n_next_pose.position.z - next_pose.position.z, 2));
        double radius = std::min(radius1, radius2);
        if (radius < 0.02)
            return 0.0;
        return radius;
    }

    moveit_msgs::MotionSequenceRequest Single_arm::construct_seq_req(std::vector<std::string> &V_name, std::vector<double> &V_radius,
                                                                     std::vector<double> &V_speed, std::vector<double> &V_acc,
                                                                     std::vector<std::string> &V_planner)
    {
        moveit_msgs::Constraints pose_goal;
        moveit_msgs::MotionPlanRequest freq, req;
        moveit_msgs::MotionSequenceItem sequenceItem;
        moveit_msgs::MotionSequenceRequest sequenceRequest;
        geometry_msgs::Pose curr_pose, next_pose, n_next_pose;

        move_group_ptr_->constructMotionPlanRequest(freq);
        move_group_ptr_->constructMotionPlanRequest(req);
        freq.group_name = move_group_ptr_->getName();
        req.group_name = move_group_ptr_->getName();
        freq.goal_constraints.resize(1);
        req.goal_constraints.resize(1);

        // 如果是验证，需要将上一条路径的最终路径点作为此次的起始点
        curr_pose = move_group_ptr_->getCurrentPose().pose;
        get_constraint(V_name[0], pose_goal, next_pose);
        freq.goal_constraints[0] = pose_goal;
        freq.max_velocity_scaling_factor = V_speed[0];
        freq.max_acceleration_scaling_factor = V_acc[0];
        freq.planner_id = V_planner[0];

        // 若多个目标路点,则计算融合半径;若只有一个目标路点,融合半径为0
        if (V_name.size() > 1)
        {
            get_constraint(V_name[1], pose_goal, n_next_pose);
            sequenceItem.blend_radius =
                get_blend_radius(curr_pose, next_pose, n_next_pose) * V_radius[0];
        }
        else
        {
            sequenceItem.blend_radius = 0;
        }
        sequenceItem.req = freq;
        sequenceRequest.items.emplace_back(sequenceItem);

        if (V_name.size() > 1)
        {
            // 构造第2~n-1个MotionPlanRequest
            int i;
            for (i = 1; i < V_name.size() - 1; ++i)
            {
                curr_pose = next_pose;
                next_pose = n_next_pose;
                req.goal_constraints[0] = pose_goal;
                req.max_velocity_scaling_factor = V_speed[i];
                req.max_acceleration_scaling_factor = V_acc[i];
                req.planner_id = V_planner[i];
                get_constraint(V_name[i + 1], pose_goal, n_next_pose);
                sequenceItem.blend_radius =
                    get_blend_radius(curr_pose, next_pose, n_next_pose) * V_radius[i];
                sequenceItem.req = req;
                sequenceRequest.items.emplace_back(sequenceItem);
            }
            // 构造第n个MotionPlanRequest
            req.goal_constraints[0] = pose_goal;
            req.max_velocity_scaling_factor = V_speed[i];
            req.max_acceleration_scaling_factor = V_acc[i];
            req.planner_id = V_planner[i];
            sequenceItem.blend_radius = 0.0;
            sequenceItem.req = req;
            sequenceRequest.items.emplace_back(sequenceItem);
        }
        return sequenceRequest;
    }

    void Single_arm::get_constraint(std::string name,
                                    moveit_msgs::Constraints &pose_goal,
                                    geometry_msgs::Pose &next_pose, JPtype jptype)
    {
        std::vector<double> joints;

        robot_state::RobotState rstate(robotmodel_ptr_);
        rstate.setToDefaultValues();

        if (jptype == JOINT)
        {

            if(!get_joint(name,joints))
                return;
            rstate.setVariablePositions(joint_names_, joints);
            pose_goal = kinematic_constraints::constructGoalConstraints(
                rstate, robotmodel_ptr_->getJointModelGroup(move_group_ptr_->getName()));
            Eigen::Isometry3d mat = rstate.getGlobalLinkTransform(move_group_ptr_->getEndEffectorLink());
            next_pose = Pose_Tool::toPose(mat);
        }
        else
        {
            if(!get_abs_pose(name,next_pose))
                return;
            if(!get_joint(name + "_inverse",joints))
                return;
            rstate.setVariablePositions(joint_names_, joints);
            rstate.setFromIK(robotmodel_ptr_->getJointModelGroup(move_group_ptr_->getName()),
                             next_pose, move_group_ptr_->getEndEffectorLink());
            pose_goal = kinematic_constraints::constructGoalConstraints(
                rstate, robotmodel_ptr_->getJointModelGroup(move_group_ptr_->getName()));
        }
    }

    /*------------------获得相关信息的函数------------------------------*/
    bool Single_arm::getJPDataModifiedParam(){
        bool value;
        if (ros::param::get("/B_record/JP_data_modified_flag", value))
            return value;
        else {
            ROS_WARN("未找到参数 /B_record/JP_data_modified_flag，使用默认值: false");
            return false;
        }
    }
    void Single_arm::setJPDataModifiedParam(bool value){
        ros::param::set("/B_record/JP_data_modified_flag", value);
    }
    /*------------------保存加载相关函数------------------------------*/
    void Single_arm::load_joint_pose(const std::string &station_name)
    {
        if(left_station_name_==nullptr){
            if(station_name.length()>=5&&station_name.substr(0,5)=="left_")
                left_station_name_=&station_name;
            else if(station_name.length()>=6&&station_name.substr(0,6)=="right_")
                right_station_name_=&station_name;
        }
        ROS_INFO("load station_name=%s",station_name.c_str());
        std::string file_root = ros::package::getPath("manipulator") + "/data/";
        Json::Reader jsonreader;
        Json::Value root;
        std::string file_path = file_root + station_name + ".json";
        std::ifstream file(file_path, std::ios::binary);
        if (jsonreader.parse(file, root))
        {
            // 获取关节角
            Json::Value value = root["joints"];
            std::string name;
            for (int i = 0; i < value.size(); ++i)
            {
                name = value[i]["name"].asString();
                joints_[name].resize(6);
                for (int j = 0; j < 6; ++j)
                    joints_[name].at(j) = value[i]["value"][j].asDouble();
            }
            // 获取位姿
            value = root["poses"];
            geometry_msgs::Pose pose;
            for (int i = 0; i < value.size(); ++i)
            {
                name = value[i]["name"].asString();
                pose.position.x = value[i]["value"][0].asDouble();
                pose.position.y = value[i]["value"][1].asDouble();
                pose.position.z = value[i]["value"][2].asDouble();
                pose.orientation.x = value[i]["value"][3].asDouble();
                pose.orientation.y = value[i]["value"][4].asDouble();
                pose.orientation.z = value[i]["value"][5].asDouble();
                pose.orientation.w = value[i]["value"][6].asDouble();
                abs_pose_[name] = pose;
            }
            // 获取站点位姿
            value = root["station_poses"];
            for (int i = 0; i < value.size(); ++i)
            {
                name = value[i]["name"].asString();
                pose.position.x = value[i]["value"][0].asDouble();
                pose.position.y = value[i]["value"][1].asDouble();
                pose.position.z = value[i]["value"][2].asDouble();
                pose.orientation.x = value[i]["value"][3].asDouble();
                pose.orientation.y = value[i]["value"][4].asDouble();
                pose.orientation.z = value[i]["value"][5].asDouble();
                pose.orientation.w = value[i]["value"][6].asDouble();
                rel_pose_[name] = pose;
            }
            // 获取marker_id
            value = root["marker"];
            station_id_[station_name] = value["id"].asInt();
        }
        setJPDataModifiedParam(false);
        file.close();
    }

    void Single_arm::reload_joint_pose(){
        if(getJPDataModifiedParam()&&left_station_name_!=nullptr){
            load_joint_pose(*left_station_name_);
            load_joint_pose(*right_station_name_);
        }
    }

    bool Single_arm::get_joint(const std::string &name,
                    std::vector<double> & joints)
    {
        reload_joint_pose();
        auto it = joints_.find(name);
        if (it == joints_.end())
        {
            ROS_ERROR_STREAM("no_joints_value:" + name);
            return false;
        }
        joints=it->second;
        return true;
    }

    bool Single_arm::get_abs_pose(const std::string &name,
                    geometry_msgs::Pose& pose)
    {
        reload_joint_pose();
        auto it = abs_pose_.find(name);
        if (it == abs_pose_.end())
        {
            ROS_ERROR_STREAM("no_absolute_pose_value:" + name);
            return false;
        }
        pose=it->second;
        return true;
    }

    bool Single_arm::get_rel_pose(const std::string &name,
                    geometry_msgs::Pose& pose)
    {
        reload_joint_pose();
        auto it = rel_pose_.find(name);
        if (it == rel_pose_.end())
        {
            ROS_ERROR_STREAM("no_relative_pose_value:" + name);
            return false;
        }
        pose=it->second;
        return true;
    }

    /*------------------保存加载相关函数------------------------------*/

    /*------------------添加/去除mesh物体函数-------------------------*/
    bool Single_arm::addobject_mesh(const std::string object_name,
                                    std::string object_address,
                                    geometry_msgs::Pose &self_mesh_pose)
    {
        // 放置障碍物信息
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_ptr_->getPlanningFrame();
        ROS_INFO_NAMED("Single_arm", "The address of object is %s", object_address.c_str());

        /*-------编辑障碍物信息--------*/
        // 设置名称
        collision_object.id = object_name;

        // mesh文件的信息来源
        shapes::Mesh *m = shapes::createMeshFromResource("file://" + object_address);
        shape_msgs::Mesh self_mesh;
        shapes::ShapeMsg self_mesh_msg;
        shapes::constructMsgFromShape(m, self_mesh_msg);
        self_mesh = boost::get<shape_msgs::Mesh>(self_mesh_msg);

        ROS_INFO_NAMED("Single_arm", "mesh_pose: %f, %f, %f, %f, %f, %f, %f",
                       self_mesh_pose.position.x,
                       self_mesh_pose.position.y,
                       self_mesh_pose.position.z,
                       self_mesh_pose.orientation.x,
                       self_mesh_pose.orientation.y,
                       self_mesh_pose.orientation.z,
                       self_mesh_pose.orientation.w);
        collision_object.meshes.push_back(self_mesh);
        collision_object.mesh_poses.push_back(self_mesh_pose);
        collision_object.operation = collision_object.ADD;

        ROS_INFO("Add an self-mesh into the world");

        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(collision_object);
        planning_scene.is_diff = true;

        planning_scene_diff_publisher_.publish(planning_scene);

        // 为放置的障碍物进行登记
        object_ids.push_back(collision_object.id);

        return true;
    }

    bool Single_arm::addobject_general(Object_type object_type,
                                       const std::string object_name,
                                       geometry_msgs::Pose &plane_pose)
    {
        moveit_msgs::CollisionObject collision_object;

        shape_msgs::Plane plane;
        shape_msgs::SolidPrimitive primitive;

        collision_object.header.frame_id = move_group_ptr_->getPlanningFrame(); // 障碍物坐标系
        collision_object.id = object_name;

        switch (object_type)
        {
        case Plane:

            plane.coef[0] = 0.0; // 平面的法向量
            plane.coef[1] = 0.0;
            plane.coef[2] = 1.0;
            plane.coef[3] = 0.0; // 平面的方程参数
            collision_object.planes.push_back(plane);
            // 设置平面的位置
            collision_object.plane_poses.push_back(plane_pose);
            // 设置操作类型为添加障碍物
            collision_object.operation = moveit_msgs::CollisionObject::ADD;
            ROS_INFO("Plane obstacle has added to the planning scene.");
            break;
        case Box:
            ROS_INFO("Box obstacle has added to the planning scene.");
            break;
        case Sphere:
            ROS_INFO("Sphere obstacle has added to the planning scene.");
            break;
        case Cylinder:
            ROS_INFO("Cylinder obstacle has added to the planning scene.");
            break;
        case Cone:
            ROS_INFO("Cone obstacle has added to the planning scene.");
            break;
        }

        // 创建PlanningScene消息并添加障碍物
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(collision_object);
        planning_scene.is_diff = true; // 表示这是一个增量更新
                                       // 发布PlanningScene消息，添加障碍物
        planning_scene_diff_publisher_.publish(planning_scene);

        // 为放置的障碍物进行登记
        object_ids.push_back(collision_object.id);

        return true;
    }

    bool Single_arm::removeobject(std::string remove_object_id)
    {
        ROS_INFO_NAMED("Single_arm", "Delete the barriers.....");
        // 创建一个 PlanningScene 消息
        moveit_msgs::PlanningScene planning_scene;

        // 创建 CollisionObject 消息，指定要删除的障碍物的 ID 列表
        moveit_msgs::CollisionObject remove_object;
        remove_object.header.frame_id = move_group_ptr_->getPlanningFrame(); // 障碍物所在的坐标系
        remove_object.id = remove_object_id;
        remove_object.operation = moveit_msgs::CollisionObject::REMOVE;

        // 设置要删除的障碍物
        planning_scene.world.collision_objects.push_back(remove_object);
        planning_scene.is_diff = true;

        // 发布 PlanningScene 消息以删除障碍物
        planning_scene_diff_publisher_.publish(planning_scene);

        ROS_INFO_NAMED("Single_arm", "Have deleted the barriers of %s", remove_object_id.c_str());
        return true;
    }

    /*------------------添加/去除mesh物体函数-------------------------*/

    /*------------------姿态/位姿约束物体函数-------------------------*/
    void Single_arm::SinglePose_Constraint(geometry_msgs::Pose con_pose, 
                                           Constraint_type con_type, 
                                           std::string apply_frame,
                                           std::string ref_frame)
    {
        moveit_msgs::Constraints move_constraints;
        moveit_msgs::OrientationConstraint ori_constraint;
        moveit_msgs::PositionConstraint posi_constraint;

        switch (con_type)
        {
        case Orientation:
            ori_constraint.header.frame_id = ref_frame;
            ori_constraint.link_name = apply_frame;
            ori_constraint.orientation = con_pose.orientation;
            ori_constraint.absolute_x_axis_tolerance = 0.1;
            ori_constraint.absolute_y_axis_tolerance = 0.1;
            ori_constraint.absolute_z_axis_tolerance = 0.1;
            ori_constraint.weight = 1.0;
            move_constraints.orientation_constraints.push_back(ori_constraint);
            break;
        case Position:
            // 定义仅约束 X 轴的位置约束
            posi_constraint.header.frame_id = ref_frame;  // 参考坐标系            
            posi_constraint.link_name = apply_frame;  // 末端执行器的链接名称
            posi_constraint.target_point_offset.x = 0.0;
            posi_constraint.target_point_offset.y = 0.0;
            posi_constraint.target_point_offset.z = 0.0;
            posi_constraint.constraint_region.primitives.resize(1);
            posi_constraint.constraint_region.primitive_poses.resize(1);
            posi_constraint.constraint_region.primitive_poses[0] = con_pose;
            posi_constraint.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
            posi_constraint.constraint_region.primitives[0].dimensions.resize(3);
            posi_constraint.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;  // 任意大值
            posi_constraint.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;  // 仅约束 Y 轴
            posi_constraint.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 10;  // 任意大值
            posi_constraint.weight = 1.0;  // 权重   
            move_constraints.position_constraints.push_back(posi_constraint);
            break;
        default:
            break;
        }
        move_group_ptr_->setPathConstraints(move_constraints);
    }

    void Single_arm::Clear_Constraint(void)
    {
        move_group_ptr_->clearPathConstraints();
    }
    /*------------------姿态/位姿约束物体函数-------------------------*/

    /*------------------------单臂区--------------------------------*/

    /*------------------------双臂区--------------------------------*/
    Dual_arm::Dual_arm(ros::NodeHandle &nh,
                       moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                       moveit_visual_tools::MoveItVisualToolsPtr vtptr) : Single_arm(nh, mgptr, vtptr)
    {
        // 赋值
        this->nh_ = nh;
        this->move_group_ptr_ = mgptr;
        this->visual_tools_ptr_ = vtptr;

        // 加载机器人模型
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robotmodel_ptr_ = robot_model_loader.getModel();

        // 获取关节组名称
        std::string robotgroup_name = move_group_ptr_->getName();
        joint_model_group = move_group_ptr_->getCurrentState()->getJointModelGroup(robotgroup_name);
        moveit::core::RobotStatePtr current_state = move_group_ptr_->getCurrentState();

        // 获取规划组的子规划组名称列表
        const std::vector<std::string> &subgroup_names = joint_model_group->getSubgroupNames();
        // 定义两个子规划组（一定要注意，哪个是left，哪个是right）
        this->left_mgtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(subgroup_names[0]);
        this->right_mgtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(subgroup_names[1]);

        // 获得关节角名称
        std::vector<std::string> temp_joint_names_ = move_group_ptr_->getJoints();
        for (size_t i = 0; i < temp_joint_names_.size(); i++)
        {
            size_t temp_i = i % 9;
            if ((temp_i == 0) || temp_i > 6)
                continue;
            else
                joint_names_.push_back(temp_joint_names_[i]);
        }
        left_joint_names_.insert(left_joint_names_.end(), joint_names_.begin(), joint_names_.begin() + 6);
        right_joint_names_.insert(right_joint_names_.end(), joint_names_.begin() + 6, joint_names_.end());

        // 记录初始关节
        current_state->copyJointGroupPositions(joint_model_group, Initial_joint_group);

        ROS_INFO_NAMED("Dual_arm", "Dual_arm: All is in order!");
    }

    Dual_arm::~Dual_arm()
    {
        ROS_INFO_NAMED("Dual_arm", "Dual_arm: This Class is to free.....");
    }

    /*------------------------双臂区--------------------------------*/
    /*----------------------运动规划函数-----------------------------*/
    bool Dual_arm::move_ptp(std::string left_name, std::string right_name,
                            JPtype jptype, bool execute_flag)
    {
        move_group_ptr_->setPlannerId("PTP");
        std::vector<double> dual_joints, left_joints, right_joints;
        geometry_msgs::Pose left_abs_pose,right_abs_pose;

        move_group_ptr_->setStartStateToCurrentState();
        switch (jptype)
        {
        case JOINT:
            if(!get_joint(left_name,left_joints))
                return false;
            if(!get_joint(right_name,right_joints))
                return false;
            dual_joints.insert(dual_joints.end(), left_joints.begin(), left_joints.end());
            dual_joints.insert(dual_joints.end(), right_joints.begin(), right_joints.end());

            move_group_ptr_->setJointValueTarget(dual_joints);
            break;

        case POINT:
            if(!get_joint(left_name + "_inverse",left_joints))
                return false;
            if(!get_joint(right_name + "_inverse",right_joints))
                return false;
            if(!get_abs_pose(left_name,left_abs_pose))
                return false;
            if(!get_abs_pose(right_name,right_abs_pose))
                return false;
            dual_joints.insert(dual_joints.end(), left_joints.begin(), left_joints.end());
            dual_joints.insert(dual_joints.end(), right_joints.begin(), right_joints.end());

            robot_state::RobotState rstate(robotmodel_ptr_);
            rstate.setToDefaultValues();
            rstate.setVariablePositions(joint_names_, dual_joints);
            
            rstate.setFromIK(robotmodel_ptr_->getJointModelGroup(left_mgtr->getName()),
                             left_abs_pose, left_mgtr->getEndEffectorLink());
            rstate.setFromIK(robotmodel_ptr_->getJointModelGroup(right_mgtr->getName()),
                             right_abs_pose, right_mgtr->getEndEffectorLink());

            move_group_ptr_->setJointValueTarget(rstate);
            break;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_ptr_->plan(plan) !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR_STREAM("Default pose can not be reached!");
            return false;
        }

        // 规划路径赋值
        my_plan_ = plan;
        // 将规划出来的路径通过rviz可视化
        visual_tools_ptr_->publishTrajectoryLine(my_plan_.trajectory_, joint_model_group);
        visual_tools_ptr_->trigger();

        if (execute_flag)
        {
            ROS_INFO_NAMED("Single_arm", "Planning and execution of the movement succeeded!");
            move_group_ptr_->execute(my_plan_);
        }
        else
        {
            ROS_INFO_NAMED("Single_arm", "Only planning of the movement realizes!");
        }
        visual_tools_ptr_->deleteAllMarkers();
        return true;
    }

    bool Dual_arm::move_targetPose(geometry_msgs::Pose &left_target_pose,
                                   geometry_msgs::Pose &right_target_pose, bool execute_flag, bool echo, int retry)
    {
        if(echo)ROS_INFO_NAMED("Dual_arm", "Will reach  the target pose...");
        move_group_ptr_->setStartStateToCurrentState();

        // 设置双臂目标
        move_group_ptr_->setPoseTarget(left_target_pose, left_mgtr->getEndEffectorLink());
        move_group_ptr_->setPoseTarget(right_target_pose, right_mgtr->getEndEffectorLink());

        /*------------------规划和执行运动--------------------*/
        // 规划成功标志
        bool success;
        success = (move_group_ptr_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
        // 将规划出来的路径通过rviz可视化
        visual_tools_ptr_->publishTrajectoryLine(my_plan_.trajectory_, joint_model_group);
        visual_tools_ptr_->trigger();
        my_plan_accessed = my_plan_;

        /*----------------------------------------*/

        if (success)
        {
            if (execute_flag)
            {
                if(echo)ROS_INFO_NAMED("Dual_arm", "Planning and execution of the movement succeeded!");
                move_group_ptr_->execute(my_plan_);
            }
            else
            {
                if(echo)ROS_INFO_NAMED("Dual_arm", "Only planning of the movement realizes!");
            }
            visual_tools_ptr_->deleteAllMarkers();
            return true;
        }
        else
        {
            ROS_ERROR_NAMED("Dual_arm", "Planning of the movement failed! (%d)",retry);
            visual_tools_ptr_->deleteAllMarkers();
            if(retry--){
                ROS_WARN_NAMED("Dual_arm", "Retry planning the movement!");
                return move_targetPose(left_target_pose,right_target_pose,execute_flag,echo,retry);
            }
            return false;
        }
    }

    bool Dual_arm::move_targetJoints(std::vector<double> &left_joint_pointions,
                                     std::vector<double> &right_joint_pointions, bool execute_flag, bool echo, int retry)
    {
        if(echo)ROS_INFO_NAMED("Dual_arm", "Will reach  the target joints...");

        move_group_ptr_->setStartStateToCurrentState();

        std::vector<double> dual_joint_pointions;
        dual_joint_pointions.insert(dual_joint_pointions.end(), left_joint_pointions.begin(), left_joint_pointions.end());
        dual_joint_pointions.insert(dual_joint_pointions.end(), right_joint_pointions.begin(), right_joint_pointions.end());

        move_group_ptr_->setJointValueTarget(dual_joint_pointions);
        /*------------------规划和执行运动--------------------*/
        // 规划成功标志
        bool success;
        success = (move_group_ptr_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
        // 将规划出来的路径通过rviz可视化
        visual_tools_ptr_->publishTrajectoryLine(my_plan_.trajectory_, joint_model_group);
        visual_tools_ptr_->trigger();
        my_plan_accessed = my_plan_;

        /*----------------------------------------*/
        if (success)
        {
            if (execute_flag)
            {
                moveit::core::MoveItErrorCode execute_result = move_group_ptr_->execute(my_plan_);
                // 判断执行是否成功
                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    if(echo)ROS_INFO_NAMED("Dual_arm", "Planning and execution of the movement succeeded!");
                }
                else
                {
                    ROS_ERROR_NAMED("Dual_arm", "Execution failed! Error code: %d", execute_result.val);
                    visual_tools_ptr_->deleteAllMarkers();
                    return false; // 执行失败返回false
                }
            }
            else
            {
                if(echo)ROS_INFO_NAMED("Dual_arm", "Only planning of the movement realizes!");
            }
            visual_tools_ptr_->deleteAllMarkers();
            return true;
        }
        else
        {
            ROS_ERROR_NAMED("Dual_arm", "Planning of the movement failed! (%d)",retry);
            visual_tools_ptr_->deleteAllMarkers();
            if(retry--){
                ROS_WARN_NAMED("Dual_arm", "Retry planning the movement!");
                return move_targetJoints(left_joint_pointions,right_joint_pointions,execute_flag,echo,retry);
            }
            return false;
        }
    }

    /*----------------------运动规划函数-----------------------------*/

    /*----------------------特殊动作函数-----------------------------*/
    void Dual_arm::Gel_strech_motion(double speed_scale, double acc_scale,
                                     double length[3], bool execute_flag)
    {      
        geometry_msgs::Pose left_current_pose,  right_current_pose;
        left_current_pose = left_mgtr->getCurrentPose().pose;
        right_current_pose = right_mgtr->getCurrentPose().pose;

        geometry_msgs::Pose left_goal_pose,  right_goal_pose;
        left_goal_pose = Operate_tool::tool0_usePose(length, left_current_pose);
        right_goal_pose = Operate_tool::tool0_usePose(length, right_current_pose);

        // 进行姿态约束
        this -> SinglePose_Constraint(left_current_pose, Robot_arm::Orientation, "left_tool0");
        this -> SinglePose_Constraint(right_current_pose, Robot_arm::Orientation, "right_tool0");

        move_group_ptr_->setMaxVelocityScalingFactor(speed_scale);
        move_group_ptr_->setMaxAccelerationScalingFactor(acc_scale);
        move_group_ptr_->setPlannerId("RRTstar");
        move_group_ptr_->setPlanningTime(2.0);
        move_group_ptr_->setNumPlanningAttempts(20);
        this -> move_targetPose(left_goal_pose, right_goal_pose, execute_flag);

        this -> Clear_Constraint();

        move_group_ptr_->setPlannerId("RRTConnect");
        move_group_ptr_->setPlanningTime(10.0);
    }

}
