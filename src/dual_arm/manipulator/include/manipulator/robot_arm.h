#pragma once

// 该文件将为使用Pliz规划库提供帮助
#include "manipulator/headfile.h"

namespace Robot_arm
{
/*-----Pliz声明区-------*/
#define CLOSE_SPEED 0.1
#define CLOSE_ACC 0.05
#define NORMAL_SPEED 0.3
#define NORMAL_ACC 0.2

    /*-------枚举区--------*/
    enum Movetowards
    {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        FORWARD,
        BACKWARD
    };
    enum CirType
    {
        CENTER,
        INTERIM
    };
    enum JPtype
    {
        JOINT,
        POINT
    };

    enum Object_type
    {
        Box,
        Plane,
        Sphere,
        Cylinder,
        Cone
    };

    enum Constraint_type
    {
        Position,
        Orientation
    };

    /*------------------------单臂区--------------------------------*/
    class Single_arm
    {
    public:
        /*---------------公共函数区------------------*/
        Single_arm(ros::NodeHandle &nh,
                   moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                   moveit_visual_tools::MoveItVisualToolsPtr vtptr = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world"));
        ~Single_arm();

        /*------------------运动规划函数------------------------------*/
        /**
         * @brief 点到点运动（只能指定json文件以记录的点）
         *
         * @param name 目标位姿或关节名称
         * @param JPtype 关节形式还是位姿形式（目前point是个幌子，本质还是JOINT）
         */
        bool move_ptp(std::string name, JPtype jptype, bool execute_flag = true);

        /**
         * @brief 直线运动
         *
         * @param name 目标位姿或关节名称
         */
        bool move_line(Movetowards towards, double dis, double speed_factor, bool execute_flag = true);

        /**
         * @brief 圆弧运动
         * @param type
         * 中间点类型.CENTER：指定圆心，则会在起点和目标之间按照较短的弧运动；INTERIM：指定圆弧点，则圆弧轨迹会经过指定的中间点
         * @param auxiliary_point 中间点名称
         * @param goal_point 目标点名称
         */
        bool move_circ(CirType type, std::string auxiliary_point, std::string goal_point, bool execute_flag = true);

        /**
         * @brief 根据设定的操作序列进行移动(只支持包含ptp和lin运动)
         *
         * @param V_name 需要经过的路点名称
         * @param V_radius
         * 指定每个路点的融合半径系数(0,1)(最后一个路径点必须为0)。如下图所示,真实融合半径=融合半径系数*min(||Pm-1Pm||,||PmPm+1||)
         *  @image html blend.png "平滑过渡段示意图"
         * @param V_speed
         * 指定每个路点的速度缩放系数(0,1]。实际运动速度=速度缩放系数*最大运动速度
         * @param V_acc
         * 指定每个路点的加速度缩放系数(0,1]。实际运动加速度=加速度缩放系数*最大运动加速度
         * @param V_planner
         * 指定每个路点的规划器(PTP,LIN)。PTP为点到点规划，LIN为直线规划
         */
        void move_seq(std::vector<std::string> &V_name, std::vector<double> &V_radius,
                      std::vector<double> &V_speed, std::vector<double> &V_acc,
                      std::vector<std::string> &V_planner,
                      bool execute_flag = true);

        // Pliz混合轨迹
        void pliz_seqmove(void);
        void pliz_seqmove_plus(int target_order[], int array_name, bool execute_flag);
        /**
         * @brief 运行到目标位置
         *
         * @param target_pose 目标位姿
         * @param execute_flag 默认为true，即规划并执行
         * @return 规划成功否（不包括运动） 1成0败
         */
        bool move_targetPose(geometry_msgs::Pose &target_pose, bool execute_flag = true, bool echo=true, int retry=3);

        /**
         * @brief 运行到目标关节
         *
         * @param target_pose 目标位姿
         * @param execute_flag 默认为true，即规划并执行
         * @return 规划成功否（不包括运动） 1成0败
         */
        bool move_targetJoints(std::vector<double> &joint_pointions, bool execute_flag = true, bool echo=true, int retry=3);
        /*------------------运动规划函数------------------------------*/

        /*------------------获得相关信息的函数------------------------------*/
        /**
         * @brief 根据当前位置,目标位置,以及目标位置的下一位置计算融合半径
         *
         * @param curr_pose 当前位置
         * @param next_pose 目标位置
         * @param n_next_pose 目标位置的下一位置
         * @return double
         */
        double get_blend_radius(geometry_msgs::Pose &curr_pose,
                                geometry_msgs::Pose &next_pose,
                                geometry_msgs::Pose &n_next_pose);

        /**
         * @brief 根据给定的操作，构造轨迹规划的request
         *
         * @param V_name 需要经过的路点名称
         * @param V_radius 每个路点的融合半径系数(0,1)(最后一个路径点必须为0)
         * @param V_speed 每个路点的速度系数(0,1]
         * @param V_acc 每个路点的加速度系数(0,1]
         * @param V_planner 每个路点的规划器(PTP,LIN)
         * @param validate 是否仅验证该操作是否可行
         * @return moveit_msgs::MotionSequenceRequest 构造得到的轨迹规划request
         */
        moveit_msgs::MotionSequenceRequest construct_seq_req(std::vector<std::string> &V_name, std::vector<double> &V_radius,
                                                             std::vector<double> &V_speed, std::vector<double> &V_acc,
                                                             std::vector<std::string> &V_planner);

        /**
         * @brief 根据指定名称,计算下一个目标位置和约束
         *
         * @param name 指定路点名
         * @param pose_goal 需要构造的目标约束
         * @param next_pose 需要构造的目标位置
         * @param jptype "JOINT"开头则为关节角,"POINT"开头则为位置
         */
        void get_constraint(std::string name,
                            moveit_msgs::Constraints &pose_goal,
                            geometry_msgs::Pose &next_pose, JPtype jptype = JOINT);
        /*------------------获得相关信息的函数------------------------------*/

        /*------------------保存加载相关函数------------------------------*/
        /**
         * @brief 从文件中将指定关节和相对位姿载入到joints_和rel_pose_中
         *
         * @param station_name
         */
         
        const std::string *left_station_name_=nullptr,*right_station_name_=nullptr;
        bool getJPDataModifiedParam();
        void setJPDataModifiedParam(bool value);
        void reload_joint_pose();
        void load_joint_pose(const std::string &station_name);
        /**
         * @brief 从joints_中获取关节角信息
         *
         * @param name 关节角的名称
         * @return std::vector<double> 关节角数组
         */
        bool get_joint(const std::string &name,std::vector<double>&);

        /**
         * @brief获取指定名称的绝对位姿(相对base_link)
         *
         * @param name 位姿的名称
         * @return geometry_msgs::Pose 相对位姿
         */
        bool get_abs_pose(const std::string &name,geometry_msgs::Pose&);

        /**
         * @brief 获取指定名称的相对位姿(相对工作站参考位姿)
         *
         * @param name 位姿的名称
         * @return geometry_msgs::Pose 相对位姿
         */
        bool get_rel_pose(const std::string &name,geometry_msgs::Pose&);
        /*------------------保存加载相关函数------------------------------*/

        /*------------------添加/去除mesh物体函数-------------------------*/
        /**
         * @brief 添加自定义mesh文件
         *
         * @param object_name mesh文件的名字
         * @param object_address 自定义文件的路径地址
         * @param self_mesh_pose mesh文件的姿态
         * @return 放置成功否 1成0败
         */
        bool addobject_mesh(const std::string object_name,
                            std::string object_address,
                            geometry_msgs::Pose &self_mesh_pose);
        /**
         * @brief 添加官方mesh文件
         *
         * @param object_type 障碍物的类型
         * Box, Plane, Sphere, Cylinder, Cone
         * @param object_name 障碍物的名字
         * @param self_mesh_pose 障碍物的姿态
         * @return 放置成功否 1成0败
         */
        bool addobject_general(Object_type object_type,
                               const std::string object_name,
                               geometry_msgs::Pose &plane_pose);
        /**
         * @brief 移除已放置的障碍物
         *
         * @param object_type 障碍物的类型
         * Box, Plane, Sphere, Cylinder, Cone
         * @param object_name 障碍物的名字
         * @param self_mesh_pose 障碍物的姿态
         * @return 放置成功否 1成0败
         */
        bool removeobject(std::string object_ids);

        // 可访问的路径
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_accessed;

        /*------------------添加/去除mesh物体函数-------------------------*/

        /*------------------姿态/位姿约束物体函数-------------------------*/
        // 单臂位置、姿态约束，不适用于双臂
        void SinglePose_Constraint(geometry_msgs::Pose con_pose, 
                                   Constraint_type con_type, 
                                   std::string apply_frame,
                                   std::string ref_frame = "world");
        void Clear_Constraint(void);
        /*------------------姿态/位姿约束物体函数-------------------------*/
    private:
        ros::NodeHandle nh_;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_;
        const moveit::core::JointModelGroup *joint_model_group;

        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr_;
        moveit::core::RobotModelPtr robotmodel_ptr_;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_;

        // 记录机械臂最初的关节角，方便后续返回原位置
        std::vector<double> Initial_joint_group;
        std::vector<std::string> object_ids;

        std::vector<std::string> joint_names_;
        std::unordered_map<std::string, std::vector<double>> joints_;
        std::unordered_map<std::string, geometry_msgs::Pose> rel_pose_;
        std::unordered_map<std::string, geometry_msgs::Pose> abs_pose_;
        std::unordered_map<std::string, int> station_id_;

        ros::ServiceClient circ_client_, seq_client_;
        ros::Publisher planning_scene_diff_publisher_;
    };
    /*------------------------单臂区--------------------------------*/

    /*------------------------双臂区--------------------------------*/
    class Dual_arm : public Single_arm
    {
    public:
        Dual_arm(ros::NodeHandle &nh,
                 moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                 moveit_visual_tools::MoveItVisualToolsPtr vtptr = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world"));
        ~Dual_arm();
        /*------------------运动规划函数------------------------------*/
        /**
         * @brief 点到点运动（只能指定json文件以记录的点）
         *
         * @param name 目标位姿或关节名称
         * @param JPtype 双臂目前只支持Joint
         */
        bool move_ptp(std::string left_name, std::string right_name,
                      JPtype jptype, bool execute_flag = true);

        /**
         * @brief 运行到目标位置
         *
         * @param target_pose 目标位姿
         * @param execute_flag 默认为true，即规划并执行
         * @return 规划成功否（不包括运动） 1成0败
         */
        bool move_targetPose(geometry_msgs::Pose &left_target_pose,
                             geometry_msgs::Pose &right_target_pose, bool execute_flag = true, bool echo=true, int retry=3);
        /**
         * @brief 运行到目标关节
         *
         * @param target_pose 目标位姿
         * @param execute_flag 默认为true，即规划并执行
         * @return 规划成功否（不包括运动） 1成0败
         */
        bool move_targetJoints(std::vector<double> &left_joint_pointions,
                               std::vector<double> &right_joint_pointions, bool execute_flag = true, bool echo=true, int retry=3);
        /*------------------运动规划函数------------------------------*/

        /*----------------------特殊动作函数-----------------------------*/
        /**
         * @brief 凝胶拉伸动作
         *
         * @param speed_scale 拉伸速度（m/s）
         * @param change_value 每边拉伸的距离  +为伸，-为缩
         */
        void Gel_strech_motion(double speed_scale, double acc_scale,
                               double length[3], bool execute_flag = true);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_accessed;
    private:
        ros::NodeHandle nh_;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_, left_mgtr, right_mgtr;
        const moveit::core::JointModelGroup *joint_model_group;
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr_;

        moveit::core::RobotModelPtr robotmodel_ptr_;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_;

        // 记录机械臂最初的关节角，方便后续返回原位置
        std::vector<double> Initial_joint_group;

        std::vector<std::string> joint_names_, left_joint_names_, right_joint_names_;
    };
    /*------------------------双臂区--------------------------------*/
}

namespace Robot_capsulation
{
    struct MSG
    {
        std::string id;
        std::string exper_no;
        std::string state;
        std::string detail;

        MSG()
        {
            id = "-1";
            exper_no = "-1";
            state = "idle";
            detail = "";
        }
    };

    enum EndTool
    {
        RGI,
        Pipette
    };

    class Robot_operation
    {
    private:
        tf2_ros::Buffer robot_tfBuffer_;
        tf2_ros::TransformListener robot_tfListener_;

        ros::NodeHandle nh_;
        //modbus新夹爪控制!!!!!!!!!!!
        ros::ServiceClient PGI_gripper_client;
        ros::ServiceClient RGI_gripper_client;
        //天平通信
        ros::ServiceClient balance_client;
        //重定位
        ros::ServiceClient locator_client;

        //
        std::shared_ptr<dashboardsrv_client> left_dbptr, right_dbptr, dual_dbptr;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr, left_mgtr, right_mgtr;
        std::shared_ptr<Robot_arm::Dual_arm> dual_robot_ptr;
        std::shared_ptr<Robot_arm::Single_arm> arm_robot_ptr,left_robot_ptr, right_robot_ptr;
        std::shared_ptr<Operate_tool::FT_transition> ft_transition_ptr;

        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr_;

        ros::Publisher right_controller_pub, left_controller_pub;
        ros::Publisher dual_controller_pub;

        std::unordered_map<std::string, std::vector<double>> joints_;
        std::unordered_map<std::string, geometry_msgs::Pose> abs_pose_;
        std::unordered_map<std::string, geometry_msgs::Pose> rel_pose_;
        std::unordered_map<std::string, int> station_id_;
        // 实机实验flag，默认为false
        bool Dual_arm_flag = true, Dashboard_flag = false, Locator_flag = false, ADP1000_flag = false, Balance_flag = false, R_RGI_flag = false, L_PGI140_flag = false, debug_enable_flag = false, Autostrech_flag = false;
        std::string left_station_name="",right_station_name="",station_name="";
        ros::Publisher upturntable_angle, downturntable_angle, turntable_IsInit;
        ros::Publisher adp1000_pub;
        ros::Publisher tare_force_z_pub;
        ros::Publisher left_twist_pub,right_twist_pub;
        ros::Publisher locator_topic_pub;
        ros::Subscriber left_force_z_sub;
        /////////////////////////////////////
        ros::Subscriber station_sub_,asyn_sub_;
        ros::Publisher station_pub_, asyn_pub_, opera_over_pub_;
        int operation_over_Flag = 0;
          /**
         * @brief 操作指令接收函数
         *
         * @param obsOperation_in
         */
        void station_cb(const std_msgs::StringConstPtr &stationOperation_in);
        void asyn_cb(const std_msgs::StringConstPtr &stationOperation_in);
        /////////////////////////////////////


    public:
        Robot_operation(ros::NodeHandle &nh,
                        moveit::planning_interface::MoveGroupInterfacePtr mgtr_dual,
                        moveit::planning_interface::MoveGroupInterfacePtr mgtr_left,
                        moveit::planning_interface::MoveGroupInterfacePtr mgtr_right,
                        moveit_visual_tools::MoveItVisualToolsPtr vtptr = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world"));
        ~Robot_operation();

        bool progress_over_flag = false;
        ros::Publisher left_wrench_pub,right_wrench_pub;
        bool Image_flag = false;
        //拧瓶盖---------------------------------y
        template<class T>T my_min(const T&,const T&);
        //---------------------------------
        std::string Analysis_Command(const std::string &str, bool echo=true);
        std::string code_station_func(const std::string& operation);
        std::string code_station_func_with_args(const std::string &operation, const std::vector<std::any>& args = {});
        ////////////////////////////////////////////
        std::string gripper_ctrl(ros::ServiceClient,int,int,int,int,int,int,int);
        std::string gripper_ctrl_asyn(ros::ServiceClient,int,int,int,int,int,int,int);
        std::string gripper_stop(ros::ServiceClient);
        std::string gripper_reset(ros::ServiceClient);
        //天平操作
        double Balance_read(ros::ServiceClient);
        double Balance_tare(ros::ServiceClient);

        std::string move_test(const std::vector<std::any>& args);
        std::string move_joints_test();

        std::string grab_test(const std::vector<std::any>& args);
        std::string take_test(const std::vector<std::any>& args);

        // 不定参数函数
        std::unordered_map<std::string, std::function<std::string (const std::vector<std::any>&)>> station_func_with_args_map = {
            {
                "move_test", 
                [this](const std::vector<std::any>& args) -> std::string {  // 显式指定返回类型
                    return this->move_test(args);  // 补充return 
                }
            },
            {
                "grab_test", 
                [this](const std::vector<std::any>& args) -> std::string {  // 显式指定返回类型
                    return this->grab_test(args);  // 补充return
                }
            },
            {
                "take_test", 
                [this](const std::vector<std::any>& args) -> std::string {  // 显式指定返回类型
                    return this->take_test(args);  // 补充return
                }
            },

        };

        // std::unordered_map<std::string, std::function<std::string (const std::vector<std::any>&)>> station_func_with_args_map = {
            

        // };

        // 无参数函数
        std::unordered_map<std::string, std::string (Robot_operation::*)(void)> station_func_map = {
            {"move_joints_test", &Robot_operation::move_joints_test},
        };
    };

}