#pragma once

#include "manipulator/headfile.h"

#define PI_euler 3.14

namespace Pose_Tool
{
    class Euler2Quaternion
    {
    public:
        Euler2Quaternion(double X, double Y, double Z);

        void EulerXYZ(void);
        void change_XYZ(double new_X, double new_Y, double new_Z);
        // 成员函数声明
        double getQua_x(void);
        double getQua_y(void);
        double getQua_z(void);
        double getQua_w(void);

    private:
        // X Y Z -3.14~3.14
        // 偏航 Z
        double yaw_deg;
        // 俯仰角 Y
        double pitch_deg;
        // 翻滚 X
        double roll_deg;
        // 四元数
        double value_getQua_x;
        double value_getQua_y;
        double value_getQua_z;
        double value_getQua_w;
    };

    std::vector<double> self_getRPY(geometry_msgs::Quaternion quaternion);

    Eigen::Isometry3d toMatrix(Eigen::Vector3d &X, Eigen::Quaterniond &Q);
    Eigen::Isometry3d toMatrix(Eigen::Vector3d &X, Eigen::Vector3d &YPR);
    Eigen::Isometry3d toMatrix(geometry_msgs::Pose &pose);

    geometry_msgs::Pose toPose(Eigen::Isometry3d &T);
    geometry_msgs::Pose toPose(Eigen::Vector3d &X, Eigen::Vector3d &YPR);
    geometry_msgs::Pose toPose(Eigen::Vector3d &X, Eigen::Quaterniond &Q);

    Eigen::Matrix3d rpy2R(const Eigen::Vector3d &rpy);

    geometry_msgs::Pose Pose_EulerRPY(const geometry_msgs::Pose& bowl_pose, 
                                      double EulerRPY_angle[3]);
}

namespace Record_tool
{
    extern const std::string *RT_left_station_name_,*RT_right_station_name_,*RT_station_name_;
    extern std::unordered_map<std::string, std::vector<double> > *RT_joints__;
    extern std::unordered_map<std::string, geometry_msgs::Pose> *RT_abs_pose__;
    extern std::unordered_map<std::string, geometry_msgs::Pose> *RT_rel_pose__;
    extern std::unordered_map<std::string, int> *RT_station_id__;
    
    bool getJPDataModifiedParam();
    void setJPDataModifiedParam(bool);
    bool writeJoints2File(std::string file_path, std::string name,
                          std::vector<double> joints);
    bool writePose2File(std::string file_path, std::string name,
                        geometry_msgs::Pose set_pose);
    bool writeStation2File(std::string file_path, std::string name,
                           geometry_msgs::Pose station_pose);
    geometry_msgs::Pose computeRelPose(geometry_msgs::Pose stationToworld, 
                                    geometry_msgs::Pose toolToworld);
    bool convertPose2Station(std::string file_path, geometry_msgs::Pose stationToworld);
    /*------------------保存加载相关函数------------------------------*/
    // 从robot_arm.h中copy而来，只为获得独立于机械臂的加载记录函数
    void load_joint_pose(const std::string &station_name,
                         std::unordered_map<std::string, std::vector<double>> &joints_,
                         std::unordered_map<std::string, geometry_msgs::Pose> &abs_pose_,
                         std::unordered_map<std::string, geometry_msgs::Pose> &rel_pose_,
                         std::unordered_map<std::string, int> &station_id_);
    void reload_joint_pose();

    bool get_joint(const std::string &name,
                    std::unordered_map<std::string, std::vector<double>> &joints_,
                    std::vector<double> & joints);
    bool get_abs_pose(const std::string &name,
                    std::unordered_map<std::string, geometry_msgs::Pose> &abs_pose_,
                    geometry_msgs::Pose& pose);
    bool get_rel_pose(const std::string &name,
                    std::unordered_map<std::string, geometry_msgs::Pose> &rel_pose_,
                    geometry_msgs::Pose& pose);
    /*------------------保存加载相关函数------------------------------*/

    /*------------------保存加载轨迹函数(yaml)------------------------------*/
    YAML::Node convertTrajectoryToYAML(const moveit_msgs::RobotTrajectory& trajectory);
    void saveTrajectoryToYAML(const moveit_msgs::RobotTrajectory& trajectory, const std::string& filename);
    moveit_msgs::RobotTrajectory convertYAMLToTrajectory(const YAML::Node& node);
    void loadTrajectoryFromYAML(moveit_msgs::RobotTrajectory& trajectory, const std::string& filename);
    /*------------------保存加载轨迹函数(yaml)------------------------------*/

}

namespace Operate_tool
{
    int interrupt_judge(bool enable_flag = true);
    bool switch_controller(const std::string start_controller,
                           const std::string end_controller,
                           std::string switch_goalrobot = "");
    class FT_transition
    {
    private:
        bool aheadFT_flag = false;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        geometry_msgs::TransformStamped wTot_tfStamped;
        void coordinate_transFT(geometry_msgs::WrenchStamped &msg);
        class Filter{
            std::queue<double>q;
            int window_size;
            double sum;
            public:
                Filter():window_size(5),sum(0){}
                double operator()(double val){
                    if(q.size()<window_size){
                        sum+=val;
                        q.push(val);
                        return sum/q.size();
                    }else{
                        sum=sum-q.front()+val;
                        q.pop();
                        q.push(val);
                        return sum/window_size;
                    }
                }
                double val()const{
                    return q.empty()?0:sum/q.size();
                }
                void clear(){
                    while(!q.empty())q.pop();
                    sum=0;
                }
        };
        class Maxer{
            double a[50];
            int q_size,window_size;
            public:
                Maxer():q_size(0),window_size(30){}
                double operator()(double val){
                    if(q_size<window_size){
                        a[q_size++]=val;
                    }else{
                        for(int i=1;i<window_size;i++)a[i-1]=a[i];
                        a[window_size-1]=val;
                    }
                    double mx=a[0];
                    for(int i=1;i<q_size;i++)if(mx<a[i])mx=a[i];
                    // for(int i=0;i<q_size;i++)printf("%lf,",a[i]);
                    // puts("");
                    return mx;
                }
        };
    public:
        FT_transition(ros::NodeHandle &nh);
        ~FT_transition();

        // 力矩转换模块话题订阅与发布
        ros::NodeHandle nh_;
        ros::Subscriber ft_wrench_sub_right, ft_wrench_sub_left;
        ros::Publisher ft_wrench_pub_right, ft_wrench_pub_left;

        ros::Publisher left_force_modulus_pub, left_torque_modulus_pub;
        ros::Publisher right_force_modulus_pub, right_torque_modulus_pub;
        ros::Publisher left_force_x_pub,left_force_y_pub,left_force_z_pub;
        ros::Publisher right_torque_x_pub,right_torque_y_pub,right_torque_z_pub,right_torque_z_max_pub;
        Filter left_force_x_filter,left_force_y_filter,left_force_z_filter;
        Filter right_torque_x_filter,right_torque_y_filter,right_torque_z_filter;
        Maxer right_torque_z_maxer;

        void right_FT_wrenchCallback(geometry_msgs::WrenchStamped msg);
        void left_FT_wrenchCallback(geometry_msgs::WrenchStamped msg);
    };

    class robotiq_FT_transition
    {
    private:
        /* data */
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        geometry_msgs::TransformStamped wTot_tfStamped;

        std::string fixed_robot;
        
        Eigen::Matrix3d torque_usedMatrix;
        Eigen::Vector3d F, f;
        void Gravity_correction(geometry_msgs::WrenchStamped &msg);
        
    public:
        robotiq_FT_transition(ros::NodeHandle &nh, std::string robot_name, double* length_intool);
        ~robotiq_FT_transition();

        // 力矩转换模块话题订阅与发布
        ros::NodeHandle nh_;
        ros::Subscriber robotiq_ft_wrench_sub;
        ros::Publisher robotiq_ft_wrench_pub;

        void robotiq_FT_wrenchCallback(const geometry_msgs::WrenchStamped &msg);
    };

    /**
     * @brief 坐标系转换矩阵
     *
     * @param nh 节点
     * @param init_coordinate 原坐标系
     * @param ref_coordinate 最终的目标参考坐标系
     * @param matrix 获取的矩阵
     */
    void correct_coordinate(ros::NodeHandle nh,
                            std::string init_coordinate,
                            std::string ref_coordinate,
                            tf::Matrix3x3 &matrix);
    
    /**
     * @brief 执行器夹持点，结合其尺寸，获取对应在tool0的姿态（默认）
     *
     * @param length[3] 执行器夹持点在tool0下的坐标
     * @param ref_Pose 执行器夹持点姿态
     * @param dir_flag 若dir_flag == false, 则是tool0转换到夹持器处
     * @return 此时tool0的姿态
     */
    geometry_msgs::Pose tool0_usePose(double length[3], 
                                      geometry_msgs::Pose ref_Pose,
                                      bool dir_flag = true);

    std::vector<robot_state::RobotStatePtr> omplPathToMoveItPath(const ompl::geometric::PathGeometric &path, 
                                                                 moveit::planning_interface::MoveGroupInterfacePtr mgptr);
    std::vector<robot_state::RobotStatePtr> PathToMoveItPath(const moveit::planning_interface::MoveGroupInterface::Plan &path,
                                                             moveit::planning_interface::MoveGroupInterfacePtr mgptr);                                                        
    void Smooth_trajectory(moveit_msgs::RobotTrajectory &trajectory, double vel_acc_value,
                           std::vector<robot_state::RobotStatePtr> moveit_path,
                           moveit::planning_interface::MoveGroupInterfacePtr mgptr);
    void Merge_trajectory(std::vector<robot_state::RobotStatePtr> &moveit_path,
                          std::vector<robot_state::RobotStatePtr> merged_path,
                          int startpath_point = 1);   
    void removeDuplicateTimeStamps(moveit_msgs::RobotTrajectory& trajectory);
                    
    geometry_msgs::Pose Get_LtargetPose(geometry_msgs::Pose r_target_pose,
                                        geometry_msgs::Pose l_initpose,
                                        geometry_msgs::Pose r_initpose);

}

namespace Motion_tool
{
    enum TwistMove
    {
        lINEAR_X,
        lINEAR_Y,
        lINEAR_Z,
        ANGULAR_X,
        ANGULAR_Y,
        ANGULAR_Z
    };
    const char TwistMoveNames[]="xyzRPY";
    class TwistMoveMsg{
        double v[6];
        public:
        TwistMoveMsg(){
            for(int i=0;i<6;i++)v[i]=0;
        }
        TwistMoveMsg(double x,double y,double z,double R,double P,double Y){
            v[0]=x;
            v[1]=y;
            v[2]=z;
            v[3]=R;
            v[4]=P;
            v[5]=Y;
        }
        double& operator[](int _){return v[_];}
        double operator[](int _)const{return v[_];}
        TwistMoveMsg operator +(const TwistMoveMsg& _)const{
            return TwistMoveMsg(v[0]+_[0],v[1]+_[1],v[2]+_[2],v[3]+_[3],v[4]+_[4],v[5]+_[5]);
        }
        TwistMoveMsg operator -()const{
            return TwistMoveMsg(-v[0],-v[1],-v[2],-v[3],-v[4],-v[5]);
        }
        TwistMoveMsg operator /(double x)const{
            return TwistMoveMsg(v[0]/x,v[1]/x,v[2]/x,v[3]/x,v[4]/x,v[5]/x);
        }
        TwistMoveMsg operator *(double x)const{
            return TwistMoveMsg(v[0]*x,v[1]*x,v[2]*x,v[3]*x,v[4]*x,v[5]*x);
        }
        
        TwistMoveMsg& operator *=(double x){
            for(int i=0;i<6;i++)v[i]*=x;
            return *this;
        }
        TwistMoveMsg& operator /=(double x){
            if(fabs(x)>-1e7)for(int i=0;i<6;i++)v[i]/=x;
            return *this;
        }
        double& x(){return v[0];}
        double& y(){return v[1];}
        double& z(){return v[2];}
        double& R(){return v[3];}
        double& P(){return v[4];}
        double& Y(){return v[5];}
        double mod()const{return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]+v[3]*v[3]+v[4]*v[4]+v[5]*v[5]);}
        TwistMoveMsg set(double val){
            TwistMoveMsg res;
            for(int i=0;i<6;i++)if(fabs(v[i])>1e-7){
                res[i]=val;
                if(v[i]<0)res[i]=-res[i];
            }
            return res;
        }
        void print()const{
            printf("twist:");
            for(int i=0;i<6;i++)
                printf("%.5lf ",v[i]);
            puts("");
        }
        void add(std::string str){
            for(int i=0;i<6;i++)if(TwistMoveNames[i]==str[0]){
                try{
                    v[i]=stod(str.substr(1));
                }catch(...){
                    printf("illegal msg %s\n",str.c_str());
                }
            }
        }
    };
    
    /**
     * @brief 使机械臂末端做螺旋运动
     *
     * @param mgtr 规划组的指针
     * @param screw_pitch 螺距
     * @param expected_angle 期望旋转的角度（若大于1.57，则只旋转180度）
     * @return 剩余角度（期望小于3.14，返回0，否则返回expected_angle-3.14）
     */
    double spiral_motion_tool(moveit::planning_interface::MoveGroupInterfacePtr mgtr,
                              double screw_pitch, double expected_angle, bool motor_direction,
                              bool execute_flag = true);

    /**
     * @brief 开启twist控制，通过末端线/角速度对进行臂进行控制（缺点，精度不高)
     *
     * @param nh 节点句柄
     * @param robot_name 要控制的机械臂
     * @param speed_scale 速度百分比（正数）
     * @param change_value 期望改变值
     * @param TwistMove 改变值类型(x, y, z, r, p, y)
     * @param RPY_correct 绕参考坐标系（eg：/world）RPY的x，y，z固定轴旋转角度
     *                    或绕参考坐标系，选择YPR的角度
     * @param ref_coordinate 改变值时的参考坐标系，默认为/world
     */
    void twist_motion_single(ros::NodeHandle nh, std::string robot_name,
                             double speed_scale, double change_value,
                             TwistMove twisttype,
                             std::vector<double> RPY_correct = {0, 0, 0}, std::string ref_coordinate = "/world");
    // 针对双臂协同规划
    void twist_motion_dual(ros::NodeHandle nh, std::string left_robot_name, std::string right_robot_name,
                           double left_speed_scale, double right_speed_scale,
                           double left_change_value, double right_change_value,
                           TwistMove left_twisttype, TwistMove right_twisttype,
                           std::vector<double> RPY_correct = {0, 0, 0}, std::string ref_coordinate = "/world");
    void twist_move(ros::NodeHandle nh, std::string robot_name, TwistMoveMsg msg,
                    double run_time=0.5, std::vector<double> RPY_correct = {0, 0, 0}, std::string ref_coordinate = "/world");
    void twist_publish(ros::NodeHandle nh, ros::Publisher twist_pub, TwistMoveMsg msg, double run_time=0.5);
    void twist_dual_move(ros::NodeHandle nh, TwistMoveMsg msg,
                    double run_time=0.5, std::vector<double> RPY_correct = {0, 0, 0}, std::string ref_coordinate = "/world");
}

namespace Modeling_tools
{
    class Bowl
    {
    public:
        Bowl(geometry_msgs::Pose bottom_pose, double radius);
        geometry_msgs::Pose Get_grindPose(double goal_height, double rotation_angle);

    private:
        double radius;
        geometry_msgs::Pose bottom_pose;

    };

}

namespace Kinematics_tools
{
    class SingleArm_robot
    {
    public:
        SingleArm_robot(moveit::planning_interface::MoveGroupInterfacePtr mgptr);
        // 设置关节角度
        void setJointPositions(const std::vector<double> &joint_positions);
        // 计算正向运动学
        Eigen::Affine3d forwardKinematics();
        // 计算雅可比矩阵
        Eigen::MatrixXd computeJacobian();
        // 获取当前关节角度
        std::vector<double> getCurrentJointPositions();

    private:
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_;
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        moveit::core::RobotModelPtr robot_model_;
        moveit::core::RobotStatePtr robot_state_;
        const moveit::core::JointModelGroup* joint_model_group_;
    }; 

    class DualArm_robot
    {
    public:
        DualArm_robot(moveit::planning_interface::MoveGroupInterfacePtr mgptr);
        // 设置关节角度
        void setJointPositions(const std::vector<double> &joint_positions);
        // 计算正向运动学
        Eigen::Affine3d left_forwardKinematics();
        Eigen::Affine3d right_forwardKinematics();
        // 计算雅可比矩阵
        Eigen::MatrixXd left_computeJacobian();
        Eigen::MatrixXd right_computeJacobian();
        // 获取当前关节角度
        std::vector<double> getCurrentJointPositions();

    private:
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_, left_mgtr, right_mgtr;
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        moveit::core::RobotModelPtr robot_model_;
        moveit::core::RobotStatePtr robot_state_;
        const moveit::core::JointModelGroup *joint_model_group_, *left_jmgp_, *right_jmgp_;
        Eigen::MatrixXd RW_transform, LW_transform;
    };      

    void IKinverse_tool(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                        geometry_msgs::Pose target_pose, std::vector<double> &target_joints);
}

namespace Constraint_tools
{
    class SingleArm_Constraint : public ompl::base::Constraint
    {
    public:
        SingleArm_Constraint(moveit::planning_interface::MoveGroupInterfacePtr mgptr, unsigned int cst_num);

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

        geometry_msgs::Pose forwardKinematics(const Eigen::Ref<const Eigen::VectorXd> &x) const;
        void set_currentPJ(void);

        virtual void constraint_formula(geometry_msgs::Pose endpose, std::vector<double> &return_values) const;
        virtual void jacobian_formula(Eigen::MatrixXd jacobian_matrix, geometry_msgs::Pose endpose, 
                                      Eigen::MatrixXd &return_matrix) const;

        virtual ~SingleArm_Constraint() = default;

        std::vector<double> Init_joints;
        geometry_msgs::Pose Init_pose;

        // 如果给out[0],out[1]赋值，则该num=2
        unsigned int constraint_num;

    private:
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;
        std::shared_ptr<Kinematics_tools::SingleArm_robot> single_robot_ptr;
    };

    // class DualArm_Constraint : public ompl::base::Constraint
    class DualArm_Constraint : public ompl::base::Constraint
    {
    public:
        DualArm_Constraint(moveit::planning_interface::MoveGroupInterfacePtr mgptr, unsigned int cst_num);
                
        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

        bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

        geometry_msgs::Pose forwardKinematics(const Eigen::Ref<const Eigen::VectorXd> &x, bool LR_flag) const;

        //若需重写约束条件，只需更改constraint_formula与jacobian_formula即可
        virtual void constraint_formula(geometry_msgs::Pose endpose_left, 
                                        geometry_msgs::Pose endpose_right, 
                                        std::vector<double> &return_values) const;
        virtual void jacobian_formula(Eigen::MatrixXd left_jacobian_matrix, 
                                      Eigen::MatrixXd right_jacobian_matrix, 
                                      geometry_msgs::Pose endpose_left,
                                      geometry_msgs::Pose endpose_right,
                                      Eigen::MatrixXd &return_matrix) const;

        virtual ~DualArm_Constraint() = default;

        std::vector<double> Init_joints;
        geometry_msgs::Pose Init_pose_left, Init_pose_right;

        // 如果给out[0],out[1]赋值，则该num=2
        unsigned int constraint_num;
        bool isValid(const ompl::base::State* state) const;
    private:
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;
        std::shared_ptr<Kinematics_tools::DualArm_robot> dual_robot_ptr;
        planning_scene::PlanningScenePtr planning_scene_ptr;
    };
    // 闭链约束中的S，M，H矩阵
    Eigen::MatrixXd Constraint_S(Eigen::Vector3d xyz);  // 3*3
    Eigen::MatrixXd Constraint_M(Eigen::Vector4d wxyz, bool LR_flag);  // 4*4  true为left, false为right
    Eigen::MatrixXd Constraint_H(Eigen::Vector4d wxyz);  // 3*4
    
    // mgptr应为双臂的规划组；内部创建DualArm_Constraint类完成闭链约束
    void CloseChain_cst_move(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                             std::vector<double> target_joints, double tolerance_value,
                             unsigned int cst_num, 
                             moveit_msgs::RobotTrajectory& trajectory,
                             bool execute_flag = true);
}

