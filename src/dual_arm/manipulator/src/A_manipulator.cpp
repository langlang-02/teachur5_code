#include "manipulator/headfile.h"
#include "manipulator/robot_arm.h"

// 该程序为A_manipulator.cpp的实验田

int main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "robot_manipulator");
    ros::NodeHandle nh;

    // 开启多线程
    ros::AsyncSpinner spinner(10);
    spinner.start();

    // 读取Dual_arm_flag参数
    bool Dual_arm_flag;
    if (nh.getParam("A_manipulator/Dual_arm_flag", Dual_arm_flag))
        ROS_INFO("Got param: %d", Dual_arm_flag);
    else
        ROS_ERROR("Failed to get param 'Dual_arm_flag'");

    if(Dual_arm_flag){
        // 创建规划组
        const std::string robot_name = "dual_robot";
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(robot_name);
        move_group_ptr->setMaxVelocityScalingFactor(0.05);
        move_group_ptr->setMaxAccelerationScalingFactor(0.05);

        const std::string left_robot_name = "left_robot";
        moveit::planning_interface::MoveGroupInterfacePtr left_mgtr =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(left_robot_name);
        left_mgtr->setMaxVelocityScalingFactor(0.05);
        left_mgtr->setMaxAccelerationScalingFactor(0.05);

        const std::string right_robot_name = "right_robot";
        moveit::planning_interface::MoveGroupInterfacePtr right_mgtr =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(right_robot_name);
        right_mgtr->setMaxVelocityScalingFactor(0.05);
        right_mgtr->setMaxAccelerationScalingFactor(0.05);

        Robot_capsulation::Robot_operation dual_ur5e(nh, move_group_ptr, left_mgtr, right_mgtr);
        ROS_INFO_STREAM("-------------ready to receive command---------\n");
        while ((ros::ok()))
        {
            if(dual_ur5e.progress_over_flag)
            {
                std::cout << "The progress is over......"<<std::endl;
                break;
            }
        }
    }else{
        // 创建规划组
        const std::string robot_name = "manipulator";
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(robot_name);
        move_group_ptr->setMaxVelocityScalingFactor(0.05);
        move_group_ptr->setMaxAccelerationScalingFactor(0.05);
        
        moveit::planning_interface::MoveGroupInterfacePtr left_mgtr,right_mgtr;
        Robot_capsulation::Robot_operation dual_ur5e(nh, move_group_ptr, left_mgtr, right_mgtr);
        ROS_INFO_STREAM("-------------ready to receive command---------\n");
        while ((ros::ok()))
        {
            if(dual_ur5e.progress_over_flag)
            {
                std::cout << "The progress is over......"<<std::endl;
                break;
            }
        }
    }
    
    
    

    return 0;
}
