#include "manipulator/robot_arm.h"

namespace Robot_capsulation
{
    /*------------------------单臂区--------------------------------*/
    Robot_operation::Robot_operation(ros::NodeHandle &nh,
                                     moveit::planning_interface::MoveGroupInterfacePtr mgtr_dual,
                                     moveit::planning_interface::MoveGroupInterfacePtr mgtr_left,
                                     moveit::planning_interface::MoveGroupInterfacePtr mgtr_right,
                                     moveit_visual_tools::MoveItVisualToolsPtr vtptr): 
                                     robot_tfBuffer_(), robot_tfListener_(robot_tfBuffer_)
    {
        //////////////////////////////////////////////////////////////////////////////////////////
        // 读取Dual_arm_flag参数
        if (nh.getParam("A_manipulator/Dual_arm_flag", Dual_arm_flag))
            ROS_INFO("Got param: %d", Dual_arm_flag);
        else
            ROS_ERROR("Failed to get param 'Dual_arm_flag'");
        // 读取Dashboard_flag参数
        if (nh.getParam("A_manipulator/Dashboard_flag", Dashboard_flag))
            ROS_INFO("Got param: %d", Dashboard_flag);
        else
            ROS_ERROR("Failed to get param 'Dashboard_flag'");
        // 读取Balance_flag参数
        if (nh.getParam("A_manipulator/Balance_flag", Balance_flag))
            ROS_INFO("Got param: %d", Balance_flag);

        // 读取R_RGI_flag参数
        if (nh.getParam("A_manipulator/R_RGI_flag", R_RGI_flag))
            ROS_INFO("Got param: %d", R_RGI_flag);
        else
            ROS_ERROR("Failed to get param 'PGI140_flag'");
        // 读取L_PGI140_flag参数
        if (nh.getParam("A_manipulator/L_PGI140_flag", L_PGI140_flag))
            ROS_INFO("Got param: %d", L_PGI140_flag);
        else
            ROS_ERROR("Failed to get param 'PGI140_flag'");
        if (nh.getParam("A_manipulator/ADP1000_flag", ADP1000_flag))
            ROS_INFO("Got param: %d", ADP1000_flag);
        else
            ROS_ERROR("Failed to get param 'ADP1000_flag'");    
        if (nh.getParam("A_manipulator/Locator_flag", Locator_flag))
            ROS_INFO("Got param: %d", Locator_flag);
        else
            ROS_ERROR("Failed to get param 'Locator_flag'");    
        // 读取debug_enable_flag参数
        if (nh.getParam("A_manipulator/debug_enable_flag", debug_enable_flag))
            ROS_INFO("Got param: %d", debug_enable_flag);
        else
            ROS_ERROR("Failed to get param 'debug_enable_flag'");
        // 读取Autostrech_flag参数
        if (nh.getParam("A_manipulator/Autostrech_flag", Autostrech_flag))
            ROS_INFO("Got param: %d", Autostrech_flag);
        else
            ROS_ERROR("Failed to get param 'Autostrech_flag'");
        
        // 读取Load_JPfile参数
        std::string Load_JPfile_name = "gelrobot";
        if (nh.getParam("A_manipulator/Load_JPfile", Load_JPfile_name))
            ROS_INFO("Load JPfile name: %s", Load_JPfile_name.c_str());
        else
            ROS_ERROR("Failed to load JPfile 'Load_JPfile'");

        // 读取Autostrech_flag参数
        if (nh.getParam("A_manipulator/Image_flag", Image_flag))
            ROS_INFO("Got param: %d", Image_flag);
        else
            ROS_ERROR("Failed to get param 'Image_flag'");

        if (Dashboard_flag)
        {
            if(Dual_arm_flag){
                left_dbptr = std::make_shared<dashboardsrv_client>(nh, "/left_robot");
                right_dbptr = std::make_shared<dashboardsrv_client>(nh, "/right_robot");

                left_dbptr->robot_init();
                left_dbptr->load_program("pro_1.urp\n");
                left_dbptr->stop();
                left_dbptr->play();
                left_dbptr->DO_init();

                right_dbptr->robot_init();
                right_dbptr->load_program("pro_2.urp\n");
                right_dbptr->stop();
                right_dbptr->play();
                right_dbptr->DO_init();
            }else{
                dual_dbptr = std::make_shared<dashboardsrv_client>(nh, "");

                dual_dbptr->robot_init();
                dual_dbptr->load_program("pro_1.urp\n");
                dual_dbptr->stop();
                dual_dbptr->play();
                dual_dbptr->DO_init();
            }
        }
        //夹爪modbus
        //天平modbus
        if(Balance_flag){
            ros::service::waitForService("/Balance_com");
            balance_client = nh.serviceClient<balance_com::Balance>("/Balance_com");
        }


        if (R_RGI_flag)
        {
            ros::service::waitForService("/RGI_gripper_control");
            RGI_gripper_client = nh.serviceClient<gripper_modbus::Gripper>("/RGI_gripper_control");
            std::string R_RGI_port = "ttyUSB0";
            if (nh.getParam("A_manipulator/R_RGI_port", R_RGI_port))
                ROS_INFO("R_RGI port: %s", R_RGI_port.c_str());
            else
                ROS_WARN("Failed to get param 'A_manipulator/R_RGI_port'.");
        }
        if (L_PGI140_flag)
        {
            ros::service::waitForService("/PGI_gripper_control");
            PGI_gripper_client = nh.serviceClient<gripper_modbus::Gripper>("/PGI_gripper_control");
            std::string L_PGI140_port = "ttyUSB1";
            if (nh.getParam("A_manipulator/L_PGI140_port", L_PGI140_port))
                ROS_INFO("L_PGI140_port port: %s", L_PGI140_port.c_str());
            else
                ROS_WARN("Failed to get param 'A_manipulator/L_PGI140_port'.");
        }
        if(ADP1000_flag)
        {
            adp1000_pub = nh.advertise<std_msgs::Int32>("/adp1000cmd", 10);
        }
        if (Locator_flag)
        {
            ros::service::waitForService("/locator_service");
            locator_topic_pub = nh.advertise<std_msgs::String>("/locator_topic", 10);
            locator_client = nh.serviceClient<locatornew::Location>("/locator_service");
        }
        tare_force_z_pub = nh.advertise<std_msgs::Float64>("/tare_force_z", 10);
        std_msgs::Float64 msg;
        tare_force_z_pub.publish(msg);
        left_twist_pub = nh.advertise<std_msgs::Float64MultiArray>("/left_robot/twist_param", 10);
        right_twist_pub = nh.advertise<std_msgs::Float64MultiArray>("/right_robot/twist_param", 10);
        //////////////////////////////////////////////////////////////////////////////////////////

        this->nh_ = nh;
        this->visual_tools_ptr_ = vtptr;


        move_group_ptr = mgtr_dual;
        if(Dual_arm_flag){
            dual_robot_ptr = std::make_shared<Robot_arm::Dual_arm>(nh, move_group_ptr, visual_tools_ptr_);
            left_mgtr = mgtr_left;
            left_robot_ptr = std::make_shared<Robot_arm::Single_arm>(nh, left_mgtr, visual_tools_ptr_);
            left_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
            left_mgtr->setPlannerId("PTP");
            // left_mgtr->setPlanningPipelineId("ompl");
            // left_mgtr->setPlannerId("RRTConnect");


            right_mgtr = mgtr_right;
            right_robot_ptr = std::make_shared<Robot_arm::Single_arm>(nh, right_mgtr, visual_tools_ptr_);
            right_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
            right_mgtr->setPlannerId("PTP");
            // right_mgtr->setPlanningPipelineId("ompl");
            // right_mgtr->setPlannerId("RRTConnect");
        }else{
            arm_robot_ptr = std::make_shared<Robot_arm::Single_arm>(nh, move_group_ptr, visual_tools_ptr_);
        }
        move_group_ptr->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_ptr->setPlannerId("PTP");

        // 开启力转化
        ft_transition_ptr = std::make_shared<Operate_tool::FT_transition>(nh);

        // 发布右臂目标力矩
        right_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>(
            "/right_robot/target_wrench", 1);
        // 发布左臂目标力矩
        left_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>(
            "/left_robot/target_wrench", 1);

        // 发布左臂力控话题
        left_controller_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/left_robot/selfdefined_trajectory_controller/command", 1);
        // 发布右臂力控话题
        right_controller_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/right_robot/selfdefined_trajectory_controller/command", 1);

        // 发布双臂力控话题
        dual_controller_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/dual_robot/selfdefined_trajectory_controller/command", 1);

        // 工作站回调函数
        station_sub_ = nh.subscribe<std_msgs::String>("/stationOperation_in", 1,
                                                       &Robot_operation::station_cb, this);
        asyn_sub_ = nh.subscribe<std_msgs::String>("/asynOperation_in", 1,
                                                       &Robot_operation::asyn_cb, this);
        station_pub_ = nh.advertise<std_msgs::String>("/stationOperation_out", 1);
        asyn_pub_ = nh.advertise<std_msgs::String>("/asynOperation_out", 1);

        opera_over_pub_ = nh.advertise<std_msgs::String>("/stationOperation_over", 1);

        if(Dual_arm_flag){
            Record_tool::load_joint_pose(left_station_name="left_"+Load_JPfile_name+"_JP", joints_, abs_pose_, rel_pose_, station_id_);
            Record_tool::load_joint_pose(right_station_name="right_"+Load_JPfile_name+"_JP", joints_, abs_pose_, rel_pose_, station_id_);
            if(Record_tool::RT_left_station_name_!=nullptr)
                std::cout<<"加载站点数据文件:"<<Record_tool::RT_left_station_name_<<','<<*Record_tool::RT_left_station_name_<<std::endl;
            if(Record_tool::RT_right_station_name_!=nullptr)
                std::cout<<"加载站点数据文件:"<<Record_tool::RT_right_station_name_<<','<<*Record_tool::RT_right_station_name_<<std::endl;
        }else{
            Record_tool::load_joint_pose(station_name=Load_JPfile_name+"_JP", joints_, abs_pose_, rel_pose_, station_id_);
            if(Record_tool::RT_station_name_!=nullptr)
                std::cout<<"加载站点数据文件:"<<Record_tool::RT_station_name_<<','<<*Record_tool::RT_station_name_<<std::endl;

        }
    }

    Robot_operation::~Robot_operation()
    {
        ROS_INFO_NAMED("Robot_operation", "The operation of Robot is to free.....");
        if (Dashboard_flag)
        {
            if(Dual_arm_flag){
                left_dbptr->stop();
                right_dbptr->stop();
            }else{
                dual_dbptr->stop();
            }
        }
    }

    void Robot_operation::station_cb(const std_msgs::StringConstPtr &obsOperation_in)
    {
        Json::Reader jsonreader;
        Json::Value root;
        jsonreader.parse(obsOperation_in->data, root);
        ROS_INFO_STREAM("--------------received command---------------\n"
                        << root.toStyledString());

        std::string station_name = root["curr_station"].asString();
        std::string operation = root["operation"].asString();
        std::string command = root["order"].asString();
        std::string result = Analysis_Command(command);
        if(operation_over_Flag)
        {
            std_msgs::String return_msg;
            if(~operation_over_Flag)
                return_msg.data = "{'cmd':'" + operation + "','isFinish':true}";
            else
                return_msg.data = "{'cmd':'" + operation + "','isFinish':false}";
            opera_over_pub_.publish(return_msg);
            operation_over_Flag = 0;
        }

        std_msgs::String return_msg;
        std::stringstream ss;
        ss << "{"
            << "\"curr_station\":\"" << station_name << "\","
            << "\"operation\":\"" << operation << "\","
            << "\"order\":\"" << command << "\","
            << "\"result\":\"" << result << "\""
            << "}";
        return_msg.data = ss.str();
        station_pub_.publish(return_msg);
    }
    void Robot_operation::asyn_cb(const std_msgs::StringConstPtr &obsOperation_in)
    {
        Json::Reader jsonreader;
        Json::Value root;
        jsonreader.parse(obsOperation_in->data, root);
        ROS_INFO_STREAM("--------------received asyn requery command---------------\n"
                        << root.toStyledString());

        std::string station_name = root["curr_station"].asString();
        std::string operation = root["operation"].asString();
        std::string command = root["order"].asString();
        std::string result = Analysis_Command(command);
        std_msgs::String return_msg;
        std::stringstream ss;
        ss << "{"
            << "\"curr_station\":\"" << station_name << "\","
            << "\"operation\":\"" << operation << "\","
            << "\"order\":\"" << command << "\","
            << "\"result\":\"" << result << "\""
            << "}";
        return_msg.data = ss.str();
        asyn_pub_.publish(return_msg);
    }
    std::string Robot_operation::code_station_func_with_args(const std::string &operation, const std::vector<std::any>& args) {
        if (operation == "progress_over") {
            this->progress_over_flag = true;
        } else {
            if (station_func_with_args_map.find(operation) != station_func_with_args_map.end()) {
                return station_func_with_args_map[operation](args);
            } else {
                std::cout << "Function " << operation << " not found!" << std::endl;
            }
        }return "error";
    }
    std::string Robot_operation::code_station_func(const std::string &operation)
    {
        if (operation == "progress_over")
        {
            this->progress_over_flag = true;
        }
        else
        {
            if (station_func_map.find(operation) != station_func_map.end()) 
                return (this->*station_func_map[operation])();
            else 
                std::cout << "Function " << operation << " not found!" << std::endl;
        }return "error";
    }
    std::string Robot_operation::gripper_ctrl(
        ros::ServiceClient gripper_client, 
        int force, int velocity, int position, 
        int torque, int speed, int abs_angle, int rel_angle){
        // 创建服务请求对象
        gripper_modbus::Gripper srv;
        // 设置请求参数
        srv.request.force               = force    ;//平行力值，20～100
        srv.request.velocity            = velocity ;//平行速度，1～100
        srv.request.torque              = torque      ;//旋转力值，20～100
        srv.request.speed               = speed   ;//旋转速度，1～100
        srv.request.abs_angle           = abs_angle         ;//绝对角度，-160*32768-32768～160*32678+32767
        srv.request.rel_angle           = rel_angle         ;//相对角度，-32768～32767
        srv.request.position            = position          ;//平行位置，0～1000
        srv.request.block_flag          = true              ;//阻塞标志
        srv.request.stop_flag           = false             ;//强制停止
        srv.request.reset_flag          = false             ;//初始化
        srv.request.feedback            = 0b1001001 ;       //反馈选项

        if (gripper_client.call(srv)) {
            ROS_INFO("夹爪状态: %s", srv.response.status.c_str());
            return srv.response.status;
        } else {
            ROS_ERROR("服务调用失败！");
            return "offline";
        }
    }
    std::string Robot_operation::gripper_ctrl_asyn(
        ros::ServiceClient gripper_client, 
        int force, int velocity, int position, 
        int torque, int speed, int abs_angle, int rel_angle){
        // 创建服务请求对象
        gripper_modbus::Gripper srv;
        // 设置请求参数设置请求参数
        srv.request.force               = force    ;//平行力值，20～100
        srv.request.velocity            = velocity ;//平行速度，1～100
        srv.request.torque              = torque      ;//旋转力值，20～100
        srv.request.speed               = speed   ;//旋转速度，1～100
        srv.request.abs_angle           = abs_angle         ;//绝对角度，-160*32768-32768～160*32678+32767
        srv.request.rel_angle           = rel_angle         ;//相对角度，-32768～32767
        srv.request.position            = position          ;//平行位置，0～1000
        srv.request.block_flag          = false              ;//阻塞标志
        srv.request.stop_flag           = false             ;//强制停止
        srv.request.reset_flag          = false             ;//初始化
        srv.request.feedback            = 0b1001001 ;       //反馈选项

        if (gripper_client.call(srv)) {
            ROS_INFO("夹爪状态: %s", srv.response.status.c_str());
            return srv.response.status;
        } else {
            ROS_ERROR("服务调用失败！");
            return "offline";
        }
    }
    std::string Robot_operation::gripper_stop(ros::ServiceClient gripper_client){
        // 创建服务请求对象
        gripper_modbus::Gripper srv;
        // 设置请求参数设置请求参数
        srv.request.force               = -1    ;//平行力值，20～100
        srv.request.velocity            = -1 ;//平行速度，1～100
        srv.request.torque              = -1      ;//旋转力值，20～100
        srv.request.speed               = -1   ;//旋转速度，1～100
        srv.request.abs_angle           = 99999999         ;//绝对角度，-160*32768-32768～160*32678+32767
        srv.request.rel_angle           = 0         ;//相对角度，-32768～32767
        srv.request.position            = -1          ;//平行位置，0～1000
        srv.request.block_flag          = true              ;//阻塞标志
        srv.request.stop_flag           = true             ;//强制停止
        srv.request.reset_flag          = false             ;//初始化
        srv.request.feedback            = 0b1001001 ;       //反馈选项

        if (gripper_client.call(srv)) {
            ROS_INFO("夹爪状态: %s", srv.response.status.c_str());
            return srv.response.status;
        } else {
            ROS_ERROR("服务调用失败！");
            return "offline";
        }
    }
    std::string Robot_operation::gripper_reset(ros::ServiceClient gripper_client){
        // 创建服务请求对象
        gripper_modbus::Gripper srv;
        // 设置请求参数
        srv.request.force               = -1    ;//平行力值，20～100
        srv.request.velocity            = -1 ;//平行速度，1～100
        srv.request.torque              = -1      ;//旋转力值，20～100
        srv.request.speed               = -1   ;//旋转速度，1～100
        srv.request.abs_angle           = 99999999         ;//绝对角度，-160*32768-32768～160*32678+32767
        srv.request.rel_angle           = 0         ;//相对角度，-32768～32767
        srv.request.position            = -1          ;//平行位置，0～1000
        srv.request.block_flag          = true              ;//阻塞标志
        srv.request.stop_flag           = false             ;//强制停止
        srv.request.reset_flag          = true             ;//初始化
        srv.request.feedback            = 0b1001001 ;       //反馈选项

        if (gripper_client.call(srv)) {
            ROS_INFO("夹爪状态: %s", srv.response.status.c_str());
            return srv.response.status;
        } else {
            ROS_ERROR("服务调用失败！");
            return "offline";
        }
    }
    double Robot_operation::Balance_read(ros::ServiceClient balance_client){
        balance_com::Balance srv;
        srv.request.operation = "weight";
        if (balance_client.call(srv)) {
            ROS_INFO("天平示数:%.1lf",srv.response.weight);
            return srv.response.weight;
        } else {
            ROS_ERROR("Failed to call service for read weight operation");
            return -1.0;
        }
    }
    double Robot_operation::Balance_tare(ros::ServiceClient balance_client){
        balance_com::Balance srv;
        srv.request.operation = "tare";
        if (balance_client.call(srv)&&srv.response.success) {
            ROS_INFO("天平去皮成功");
            return srv.response.weight;
        } else {
            ROS_ERROR("Failed to call service for tare operation");
            return -99999999;
        }
    }
}
