#include "manipulator/robot_arm.h"
#include <regex>
#include <sstream>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

const double pi=acos(-1);
void xyzrpyToPose(float x,float y,float z,float R,float P,float Y,geometry_msgs::Pose& res,const std::string& robot_name){
    res.position.x=x;
    res.position.y=y;
    res.position.z=z;
    tf::Quaternion Q=tf::createQuaternionFromRPY(R/180.0*pi,P/180.0*pi,Y/180.0*pi);
    res.orientation.x=Q.x();
    res.orientation.y=Q.y();
    res.orientation.z=Q.z();
    res.orientation.w=Q.w();
    ROS_INFO("%s Robot Quat qx=%6.4f qy=%6.4f qz=%6.4f qw=%6.4f\n",robot_name.c_str(),Q.x(),Q.y(),Q.z(),Q.w());
}
void quaternionToRPY(geometry_msgs::Quaternion Q,const std::string& robot_name){
    tf2::Quaternion q(Q.x,Q.y,Q.z,Q.w);
    double r,p,y;
    tf2::Matrix3x3(q).getRPY(r,p,y);
    ROS_INFO("%s Robot RPY r=%6.2f p=%6.2f y=%6.2f\n",robot_name.c_str(),r/pi*180,p/pi*180,y/pi*180);
}
namespace Robot_capsulation
{
    std::string Robot_operation::Analysis_Command(const std::string &str, bool echo)
    {
        std::smatch matches;
        std::string input = str;

        std::regex first_pattern("^(\\S+)");
        std::string first_string;
        ROS_INFO("get command: %s",str.c_str());
        if (std::regex_search(input, matches, first_pattern))
        {
            if(echo)std::cout << "Order Type: " << matches[1] << std::endl;
            first_string = matches[1];
        }
        else
        {
            if(echo)std::cout << "Type: No match found." << std::endl;
        }

        // DO控制，快换接头
        if (first_string == "C")
        {
            std::regex DO_pattern("^([a-zA-Z])\\s+([a-zA-Z])\\s+(-?\\d+)\\s+(true|false)$");
            if (std::regex_search(input, matches, DO_pattern))
            {
                std::string arm = matches[2];
                int Connector_Pin = std::stoi(matches[3].str());     // 将第二个匹配转换为整数
                bool Connector_Level = (matches[4].str() == "true"); // 将第三个匹配转换为布尔值

                if(echo)std::cout << "Arm: " << arm << std::endl;
                if(echo)std::cout << "DO Pin: " << Connector_Pin << std::endl;
                if(echo)std::cout << "DO Level: " << Connector_Level << std::endl;


                // IO操作
                if(arm=="L")left_dbptr->setDO(Connector_Pin, Connector_Level);
                else if(arm=="R")right_dbptr->setDO(Connector_Pin, Connector_Level);
                else if(arm=="S")dual_dbptr->setDO(Connector_Pin, Connector_Level);
                return "success";
            }
            else
            {
                if(echo)std::cout << "Connector: No match found." << std::endl;
                return "error";
            }
        }
        // 执行器控制，夹爪、移液枪
        else if (first_string == "G")
        {
            std::regex PGI_ADP_pattern("^([a-zA-Z])\\s+([+-]?\\d+)\\s+([+-]?\\d+)$");
            std::regex PGI_RGI_pattern("^([a-zA-Z])\\s+([+-]?\\d+)\\s+([+-]?\\d+)\\s+([+-]?\\d+)\\s+([+-]?\\d+)$");
            if (std::regex_search(input, matches, PGI_ADP_pattern))
            {
                //parallel_force,parallel_velocity,position,rotate_force,rotate_velocity,abs_angle,rel_angle
                int pgi_position = std::stoi(matches[2].str());
                int adp_val = std::stoi(matches[3].str());
                std::string PGI_result="idle";
                if (L_PGI140_flag){
                    // if(pgi_position>=0&&pgi_position<=1000)
                    PGI_result=gripper_ctrl(PGI_gripper_client,-1,-1,pgi_position,-1,-1,-1,-1);
                    if(echo)std::cout << "Left PGI Gripper: " << pgi_position << std::endl;
                }else{
                    if(echo)std::cout << "Left PGI Gripper: Not open." << std::endl;
                    return "error";
                }

                if (ADP1000_flag||true){
                    if(adp_val>=-5000&&adp_val<=5000){
                        if(echo)std::cout << "Adp1000: " << adp_val << std::endl;
                        std_msgs::Int32 ADP1000_msg;
                        ADP1000_msg.data = adp_val;
                        adp1000_pub.publish(ADP1000_msg);
                    }else
                        if(echo)std::cout << "Adp1000 Illegal value" << std::endl;
                }
                else{
                    if(echo)std::cout << "Adp1000: Not open." << std::endl;
                    return "error";
                }return PGI_result;
            }else if (std::regex_search(input, matches, PGI_RGI_pattern)){
                //parallel_force,parallel_velocity,position,rotate_force,rotate_velocity,abs_angle,rel_angle
                int pgi_position = std::stoi(matches[2].str());
                int RGI_position = std::stoi(matches[3].str());
                int RGI_rel_angle = std::stoi(matches[4].str());
                int RGI_abs_angle = std::stoi(matches[5].str());
                std::string PGI_result="idle",RGI_result="idle";
                if (L_PGI140_flag){
                    // if(pgi_position>=0&&pgi_position<=1000)
                    PGI_result=gripper_ctrl(PGI_gripper_client,-1,-1,pgi_position,-1,-1,-1,-1);
                    if(echo)std::cout << "Left PGI Gripper: " << pgi_position << std::endl;
                }else{
                    if(echo)std::cout << "Left PGI Gripper: Not open." << std::endl;
                    return "error";
                }

                if (R_RGI_flag){
                    RGI_result=gripper_ctrl(RGI_gripper_client,-1,-1,RGI_position,-1,-1,RGI_abs_angle,RGI_rel_angle);
                    if(echo)std::cout << "Right RGI Gripper: " << RGI_position <<" , "<< RGI_rel_angle <<" , "<< RGI_abs_angle << std::endl;
                }
                else{
                    if(echo)std::cout << "RGI: Not open." << std::endl;
                    return "error";
                }return PGI_result+" "+RGI_result;
            }
            else
            {
                if(echo)std::cout << "End Tools: No match found." << std::endl;
                return "error";
            }
        }
        // 夹爪运动参数设置
        else if (first_string == "g")
        {
            std::regex PGI_RGI_SET_pattern("^([a-zA-Z])\\s+([+-]?\\d+)\\s+([+-]?\\d+)\\s+([+-]?\\d+)\\s+([+-]?\\d+)\\s+([+-]?\\d+)\\s+([+-]?\\d+)$");
            if (std::regex_search(input, matches, PGI_RGI_SET_pattern)){
                //parallel_force,parallel_velocity,position,rotate_force,rotate_velocity,abs_angle,rel_angle
                int pgi_force = std::stoi(matches[2].str());
                int pgi_velocity = std::stoi(matches[3].str());
                int RGI_force = std::stoi(matches[4].str());
                int RGI_velocity = std::stoi(matches[5].str());
                int RGI_torque = std::stoi(matches[6].str());
                int RGI_speed = std::stoi(matches[7].str());
                std::string PGI_result="idle",RGI_result="idle";
                if (L_PGI140_flag){
                    PGI_result=gripper_ctrl(PGI_gripper_client,pgi_force,pgi_velocity,-1,-1,-1,-1,-1);
                    if(echo)std::cout << "Left PGI Gripper: force=" << pgi_force << " velocity=" << pgi_velocity << std::endl;
                }else{
                    if(echo)std::cout << "Left PGI Gripper: Not open." << std::endl;
                    return "error";
                }

                if (R_RGI_flag){
                    RGI_result=gripper_ctrl(RGI_gripper_client,RGI_force,RGI_velocity,-1,RGI_torque,RGI_speed,99999999,0);
                    if(echo){
                        std::cout << "Right RGI Gripper: force=" << RGI_force << " velocity=" << RGI_velocity << std::endl;
                        std::cout << "Right RGI Gripper: torque=" << RGI_torque << " speed=" << RGI_speed << std::endl;
                    }
                }
                else{
                    if(echo)std::cout << "RGI: Not open." << std::endl;
                    return "error";
                }return PGI_result+" "+RGI_result;
            }
            else
            {
                if(echo)std::cout << "End Tools: No match found." << std::endl;
                return "error";
            }
        }
        // 天平控制，度数、去皮
        else if (first_string == "B")
        {
            std::regex Balance_pattern("^([a-zA-Z])\\s+([a-zA-Z0-9_]+)$");
            if (std::regex_search(input, matches, Balance_pattern))
            {
                std::string operation=matches[2];
                if(echo)std::cout << "Balance pattern: " << operation << std::endl;
                if(operation=="tare")return std::to_string(Balance_tare(balance_client));
                else if(operation=="read")return std::to_string(Balance_read(balance_client));
                else if(echo)std::cout << "Unknow balance operation:" << operation << std::endl;
            }
            else
            {
                if(echo)std::cout << "Balance operation: No match found." << std::endl;
                return "error";
            }
            return "error";
        }
        // 移液枪控制
        else if (first_string == "T")//移液枪
        {
            std::regex Gripper_pattern("^([a-zA-Z])\\s+(\\d+)$");
            if (std::regex_search(input, matches, Gripper_pattern))
            {

                int Adp = std::stoi(matches[2].str());
                if (ADP1000_flag&&-10000<=Adp&&Adp<=10000)
                {
                    if(echo)std::cout << "Adp10ml: " << Adp << std::endl;
                    std_msgs::Int32 ADP1000_msg;
                    ADP1000_msg.data = Adp;
                    adp1000_pub.publish(ADP1000_msg);
                    return "success";
                }
                else
                    if(echo)std::cout << "Adp10ml: No open." << std::endl;
            }
            else
            {
                if(echo)std::cout << "Adp10ml: No match found." << std::endl;
            }return "error";
        }
        // 机械臂运动，关节点/姿态点名称解析
        else if (first_string == "A")
        {
            std::regex second_pattern("^\\S+\\s+(\\S+)");
            std::string second_string;
            std::string result="error";
            if (std::regex_search(input, matches, second_pattern))
            {
                if(echo)std::cout << "Goal Robot: " << matches[1] << std::endl;
                second_string = matches[1];
            }
            else
            {
                if(echo)std::cout << "Num: No match found." << std::endl;
                return "error";
            }

            if (second_string == "D")
            {
                std::regex Dual_pattern("^([a-zA-Z])\\s+([A-Z])\\s+([A-Z]+)\\s+([A-Z])\\s+([0-9]*\\.?[0-9]+)\\s+([0-9]*\\.?[0-9]+)\\s+([a-zA-Z0-9_]+)\\s+([a-zA-Z0-9_]+)$");
                if (std::regex_search(input, matches, Dual_pattern))
                {
                    if(echo)std::cout << "Planer Name: " << matches[3] << std::endl;
                    if(echo)std::cout << "Move Mode: " << matches[4] << std::endl;

                    if(echo)std::cout << "Move vel: " << matches[5] << std::endl;
                    if(echo)std::cout << "Move acc: " << matches[6] << std::endl;
                    if(echo)std::cout << "L Record P/J: " << matches[7] << std::endl;
                    if(echo)std::cout << "R Record P/J: " << matches[8] << std::endl;

                    if (matches[3] == "RRT")
                    {
                        move_group_ptr->setPlanningPipelineId("ompl");
                        move_group_ptr->setPlannerId("RRT");
                    }
                    else if (matches[3] == "PTP")
                    {
                        move_group_ptr->setPlanningPipelineId("pilz_industrial_motion_planner");
                        move_group_ptr->setPlannerId("PTP");
                    }
                    else if (matches[3] == "LIN")
                    {
                        ROS_WARN("Dual arm can't use LIN planner");
                        return "error";
                    }
                    else{
                        ROS_WARN("No planner %s found",matches[3].str());
                        return "error";
                    }

                    float Move_vel = std::stof(matches[5].str())*2;
                    float Move_acc = std::stof(matches[6].str());
                    move_group_ptr->setMaxVelocityScalingFactor(Move_vel);
                    move_group_ptr->setMaxAccelerationScalingFactor(Move_acc);

                    std::string left_movename, right_movename;
                    if (matches[4] == "P")
                    {
                        left_movename = "left_" + matches[7].str();
                        right_movename = "right_" + matches[8].str();
                        geometry_msgs::Pose left_pose, right_pose;
                        if(!Record_tool::get_abs_pose(left_movename, abs_pose_, left_pose))
                            return "error";
                        if(!Record_tool::get_abs_pose(right_movename, abs_pose_, right_pose))
                            return "error";
                        if(dual_robot_ptr->move_targetPose(left_pose, right_pose,true, echo))
                            return "success";
                    }
                    else if (matches[4] == "J")
                    {
                        left_movename = "left_" + matches[7].str() + "_inverse";
                        right_movename = "right_" + matches[8].str() + "_inverse";
                        std::vector<double> left_joint_inverse, right_joint_inverse;
                        if(!Record_tool::get_joint(left_movename, joints_, left_joint_inverse))
                            return "error";
                        if(!Record_tool::get_joint(right_movename, joints_, right_joint_inverse))
                            return "error";
                        if(dual_robot_ptr->move_targetJoints(left_joint_inverse, right_joint_inverse,true, echo))
                            return "success";
                    }
                    else if (matches[4] == "S") //站点姿态点
                    {
                        // 1. 获得此时站点到world的位姿
                        boost::shared_ptr<geometry_msgs::PoseStamped const> model_posestamped = 
                                    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/obj_to_robot_holdon", this->nh_, ros::Duration(5.0));
                        geometry_msgs::Pose mesh_pose;
                        if (model_posestamped != nullptr) {
                            ROS_INFO("Received PoseStamped message:");
                            ROS_INFO("Position - x: %f, y: %f, z: %f", model_posestamped->pose.position.x, 
                                                                    model_posestamped->pose.position.y, 
                                                                    model_posestamped->pose.position.z);
                            ROS_INFO("Orientation - x: %f, y: %f, z: %f, w: %f", 
                                                                    model_posestamped->pose.orientation.x, 
                                                                    model_posestamped->pose.orientation.y, 
                                                                    model_posestamped->pose.orientation.z, 
                                                                    model_posestamped->pose.orientation.w);
                            mesh_pose = model_posestamped->pose;
                        }else{
                            ROS_INFO("Failed to receive PoseStamped message.");
                            return "error";
                        }

                        // 2. 获得站点内位姿的数据
                        left_movename = "left_" + matches[7].str() + "_station";
                        right_movename = "right_" + matches[8].str() + "_station";
                        geometry_msgs::Pose left_pose, right_pose;
                        if(!Record_tool::get_rel_pose(left_movename, rel_pose_, left_pose))
                            return "error";
                        if(!Record_tool::get_rel_pose(right_movename, rel_pose_, right_pose))
                            return "error";
                        
                        // 3. 获得站内位姿在world内——位姿
                        tf2::Transform T_right_pose, T_left_pose, T_mesh_pose;
                        tf2::fromMsg(left_pose, T_left_pose);
                        tf2::fromMsg(right_pose, T_right_pose);
                        tf2::fromMsg(mesh_pose, T_mesh_pose);

                        tf2::Transform T_Ltool_world = T_mesh_pose * T_left_pose;
                        tf2::Transform T_Rtool_world = T_mesh_pose * T_right_pose;
                        geometry_msgs::Pose Ltool_world_pose, Rtool_world_pose;

                        Ltool_world_pose.position.x = T_Ltool_world.getOrigin().x();
                        Ltool_world_pose.position.y = T_Ltool_world.getOrigin().y();
                        Ltool_world_pose.position.z = T_Ltool_world.getOrigin().z();
                        Ltool_world_pose.orientation.x = T_Ltool_world.getRotation().x();
                        Ltool_world_pose.orientation.y = T_Ltool_world.getRotation().y();
                        Ltool_world_pose.orientation.z = T_Ltool_world.getRotation().z();
                        Ltool_world_pose.orientation.w = T_Ltool_world.getRotation().w();

                        Rtool_world_pose.position.x = T_Rtool_world.getOrigin().x();
                        Rtool_world_pose.position.y = T_Rtool_world.getOrigin().y();
                        Rtool_world_pose.position.z = T_Rtool_world.getOrigin().z();
                        Rtool_world_pose.orientation.x = T_Rtool_world.getRotation().x();
                        Rtool_world_pose.orientation.y = T_Rtool_world.getRotation().y();
                        Rtool_world_pose.orientation.z = T_Rtool_world.getRotation().z();
                        Rtool_world_pose.orientation.w = T_Rtool_world.getRotation().w();

                        if(dual_robot_ptr->move_targetPose(Ltool_world_pose, Rtool_world_pose))
                            return "success";
                    }
                }
                else
                {
                    if(echo)std::cout << "Dual Arm: No match found." << std::endl;
                    return "error";
                }
            }
            else if (second_string == "R")
            {
                std::regex Rightarm_pattern("^([a-zA-Z])\\s+([A-Z])\\s+([A-Z]+)\\s+([A-Z])\\s+([0-9]*\\.?[0-9]+)\\s+([0-9]*\\.?[0-9]+)\\s+([a-zA-Z0-9_]+)$");
                if (std::regex_search(input, matches, Rightarm_pattern))
                {
                    if(echo)std::cout << "Planer Name: " << matches[3] << std::endl;
                    if(echo)std::cout << "Move Mode: " << matches[4] << std::endl;
                    if(echo)std::cout << "Move vel: " << matches[5] << std::endl;
                    if(echo)std::cout << "Move acc: " << matches[6] << std::endl;
                    if(echo)std::cout << "Record P/J: " << matches[7] << std::endl;

                    if (matches[3] == "RRT")
                    {
                        right_mgtr->setPlanningPipelineId("ompl");
                        right_mgtr->setPlannerId("RRT");
                    }
                    else if (matches[3] == "PTP")
                    {
                        right_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
                        right_mgtr->setPlannerId("PTP");
                    }
                    else if (matches[3] == "LIN")
                    {
                        right_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
                        right_mgtr->setPlannerId("LIN");
                    }
                    else{
                        ROS_WARN("No planner %s found",matches[3].str());
                        return "error";
                    }

                    float Move_vel = std::stof(matches[5].str())*2;
                    float Move_acc = std::stof(matches[6].str());
                    right_mgtr->setMaxVelocityScalingFactor(Move_vel);
                    right_mgtr->setMaxAccelerationScalingFactor(Move_acc);

                    std::string right_movename;
                    if (matches[4] == "P")
                    {
                        right_movename = "right_" + matches[7].str();
                        geometry_msgs::Pose right_pose;
                        if(!Record_tool::get_abs_pose(right_movename, abs_pose_, right_pose))
                            return "error";
                        if(right_robot_ptr->move_targetPose(right_pose,true, echo))
                            return "success";
                    }
                    else if (matches[4] == "J")
                    {
                        right_movename = "right_" + matches[7].str() + "_inverse";
                        std::vector<double> right_joint_inverse;
                        if(!Record_tool::get_joint(right_movename, joints_, right_joint_inverse))
                            return "error";
                        if(right_robot_ptr->move_targetJoints(right_joint_inverse,true, echo))
                            return "success";
                    }
                    else if (matches[4] == "S")
                    {
                        // 1. 获得此时站点到world的位姿
                        boost::shared_ptr<geometry_msgs::PoseStamped const> model_posestamped = 
                                    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/obj_to_robot_holdon", this->nh_, ros::Duration(5.0));
                        geometry_msgs::Pose mesh_pose;
                        if (model_posestamped != nullptr) {
                            ROS_INFO("Received PoseStamped message:");
                            ROS_INFO("Position - x: %f, y: %f, z: %f", model_posestamped->pose.position.x, 
                                                                    model_posestamped->pose.position.y, 
                                                                    model_posestamped->pose.position.z);
                            ROS_INFO("Orientation - x: %f, y: %f, z: %f, w: %f", 
                                                                    model_posestamped->pose.orientation.x, 
                                                                    model_posestamped->pose.orientation.y, 
                                                                    model_posestamped->pose.orientation.z, 
                                                                    model_posestamped->pose.orientation.w);
                            mesh_pose = model_posestamped->pose;
                        }else{
                            ROS_INFO("Failed to receive PoseStamped message.");
                            return "error";
                        }
                        
                        // 2. 获得站点内位姿的数据
                        right_movename = "right_" + matches[7].str() + "_station";
                        geometry_msgs::Pose right_pose;
                        if(!Record_tool::get_rel_pose(right_movename, rel_pose_, right_pose))
                            return "error";
                        
                        tf2::Transform T_right_pose, T_mesh_pose;
                        tf2::fromMsg(right_pose, T_right_pose);
                        tf2::fromMsg(mesh_pose, T_mesh_pose);

                        // 3. 获得站内位姿在world内——位姿
                        tf2::Transform T_Rtool_world = T_mesh_pose * T_right_pose;
                        geometry_msgs::Pose Rtool_world_pose;
                        Rtool_world_pose.position.x = T_Rtool_world.getOrigin().x();
                        Rtool_world_pose.position.y = T_Rtool_world.getOrigin().y();
                        Rtool_world_pose.position.z = T_Rtool_world.getOrigin().z();
                        Rtool_world_pose.orientation.x = T_Rtool_world.getRotation().x();
                        Rtool_world_pose.orientation.y = T_Rtool_world.getRotation().y();
                        Rtool_world_pose.orientation.z = T_Rtool_world.getRotation().z();
                        Rtool_world_pose.orientation.w = T_Rtool_world.getRotation().w();

                        if(right_robot_ptr->move_targetPose(Rtool_world_pose))
                            return "success";
                    }
                }
                else
                {
                    if(echo)std::cout << "Right Arm: No match found." << std::endl;
                    return "error";
                }
            }
            else if (second_string == "L")
            {
                std::regex Leftarm_pattern("^([a-zA-Z])\\s+([A-Z])\\s+([A-Z]+)\\s+([A-Z])\\s+([0-9]*\\.?[0-9]+)\\s+([0-9]*\\.?[0-9]+)\\s+([a-zA-Z0-9_]+)$");
                if (std::regex_search(input, matches, Leftarm_pattern))
                {
                    if(echo)std::cout << "Planer Num: " << matches[3] << std::endl;
                    if(echo)std::cout << "Move Mode: " << matches[4] << std::endl;
                    if(echo)std::cout << "Move vel: " << matches[5] << std::endl;
                    if(echo)std::cout << "Move acc: " << matches[6] << std::endl;
                    if(echo)std::cout << "Record P/J: " << matches[7] << std::endl;

                    if (matches[3] == "RRT")
                    {
                        left_mgtr->setPlanningPipelineId("ompl");
                        left_mgtr->setPlannerId("RRT");
                    }
                    else if (matches[3] == "PTP")
                    {
                        left_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
                        left_mgtr->setPlannerId("PTP");
                    }
                    else if (matches[3] == "LIN")
                    {
                        left_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
                        left_mgtr->setPlannerId("LIN");
                    }
                    else{
                        ROS_WARN("No planner %s found",matches[3].str());
                        return "error";
                    }

                    float Move_vel = std::stof(matches[5].str())*2;
                    float Move_acc = std::stof(matches[6].str());
                    left_mgtr->setMaxVelocityScalingFactor(Move_vel);
                    left_mgtr->setMaxAccelerationScalingFactor(Move_acc);

                    std::string left_movename;
                    if (matches[4] == "P")
                    {
                        left_movename = "left_" + matches[7].str();
                        geometry_msgs::Pose left_pose;
                        if(!Record_tool::get_abs_pose(left_movename, abs_pose_, left_pose))
                            return "error";
                        if(left_robot_ptr->move_targetPose(left_pose,true, echo))
                            return "success";
                    }
                    else if (matches[4] == "J")
                    {
                        left_movename = "left_" + matches[7].str() + "_inverse";
                        std::vector<double> left_joint_inverse;
                        if(!Record_tool::get_joint(left_movename, joints_, left_joint_inverse))
                            return "error";
                        if(left_robot_ptr->move_targetJoints(left_joint_inverse,true, echo))
                            return "success";
                    }
                    else if (matches[4] == "S")
                    {
                        // 1. 获得此时站点到world的位姿 bTt
                        boost::shared_ptr<geometry_msgs::PoseStamped const> model_posestamped = 
                                    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/obj_to_robot_holdon", this->nh_, ros::Duration(5.0));
                        geometry_msgs::Pose mesh_pose;
                        if (model_posestamped != nullptr) {
                            ROS_INFO("Received PoseStamped message:");
                            ROS_INFO("Position - x: %f, y: %f, z: %f", model_posestamped->pose.position.x, 
                                                                    model_posestamped->pose.position.y, 
                                                                    model_posestamped->pose.position.z);
                            ROS_INFO("Orientation - x: %f, y: %f, z: %f, w: %f", 
                                                                    model_posestamped->pose.orientation.x, 
                                                                    model_posestamped->pose.orientation.y, 
                                                                    model_posestamped->pose.orientation.z, 
                                                                    model_posestamped->pose.orientation.w);
                            mesh_pose = model_posestamped->pose;
                        }else{
                            ROS_INFO("Failed to receive PoseStamped message.");
                            return "error";
                        }

                        // 2. 获得站点内位姿的数据
                        left_movename = "left_" + matches[7].str() + "_station";
                        // left_movename = "left_dryer_move0";
                        geometry_msgs::Pose left_pose;
                        if(!Record_tool::get_rel_pose(left_movename, rel_pose_, left_pose))
                            return "error";
                        
                        // 3. 获得站内位姿在world内——位姿
                        tf2::Transform T_left_pose, T_mesh_pose;
                        tf2::fromMsg(left_pose, T_left_pose);
                        tf2::fromMsg(mesh_pose, T_mesh_pose);

                        tf2::Transform T_Ltool_world = T_mesh_pose * T_left_pose;
                        geometry_msgs::Pose Ltool_world_pose;
                        Ltool_world_pose.position.x = T_Ltool_world.getOrigin().x();
                        Ltool_world_pose.position.y = T_Ltool_world.getOrigin().y();
                        Ltool_world_pose.position.z = T_Ltool_world.getOrigin().z();
                        Ltool_world_pose.orientation.x = T_Ltool_world.getRotation().x();
                        Ltool_world_pose.orientation.y = T_Ltool_world.getRotation().y();
                        Ltool_world_pose.orientation.z = T_Ltool_world.getRotation().z();
                        Ltool_world_pose.orientation.w = T_Ltool_world.getRotation().w();

                        if(left_robot_ptr->move_targetPose(Ltool_world_pose))
                            return "success";
                    }
                }
                else
                {
                    if(echo)std::cout << "Left Arm: No match found." << std::endl;
                    return "error";
                }
            }else if (second_string == "S") //单臂
            {
                std::regex Singlearm_pattern("^([a-zA-Z])\\s+([A-Z])\\s+([A-Z]+)\\s+([A-Z])\\s+([0-9]*\\.?[0-9]+)\\s+([0-9]*\\.?[0-9]+)\\s+([a-zA-Z0-9_]+)$");
                if (std::regex_search(input, matches, Singlearm_pattern))
                {
                    if(echo)std::cout << "Planer Num: " << matches[3] << std::endl;
                    if(echo)std::cout << "Move Mode: " << matches[4] << std::endl;
                    if(echo)std::cout << "Move vel: " << matches[5] << std::endl;
                    if(echo)std::cout << "Move acc: " << matches[6] << std::endl;
                    if(echo)std::cout << "Record P/J: " << matches[7] << std::endl;

                    if (matches[3] == "RRT")
                    {
                        move_group_ptr->setPlanningPipelineId("ompl");
                        move_group_ptr->setPlannerId("RRT");
                    }
                    else if (matches[3] == "PTP")
                    {
                        move_group_ptr->setPlanningPipelineId("pilz_industrial_motion_planner");
                        move_group_ptr->setPlannerId("PTP");
                    }
                    else if (matches[3] == "LIN")
                    {
                        move_group_ptr->setPlanningPipelineId("pilz_industrial_motion_planner");
                        move_group_ptr->setPlannerId("LIN");
                    }
                    else{
                        ROS_WARN("No planner %s found",matches[3].str());
                        return "error";
                    }

                    float Move_vel = std::stof(matches[5].str())*2;
                    float Move_acc = std::stof(matches[6].str());
                    move_group_ptr->setMaxVelocityScalingFactor(Move_vel);
                    move_group_ptr->setMaxAccelerationScalingFactor(Move_acc);

                    std::string movename;
                    if (matches[4] == "P")
                    {
                        movename = matches[7].str();
                        geometry_msgs::Pose pose;
                        if(!Record_tool::get_abs_pose(movename, abs_pose_, pose))
                            return "error";
                        if(arm_robot_ptr->move_targetPose(pose,true, echo))
                            return "success";
                    }
                    else if (matches[4] == "J")
                    {
                        movename = matches[7].str() + "_inverse";
                        std::vector<double> joint_inverse;
                        if(!Record_tool::get_joint(movename, joints_, joint_inverse))
                            return "error";
                        if(arm_robot_ptr->move_targetJoints(joint_inverse,true, echo))
                            return "success";
                    }
                    else if (matches[4] == "S")
                    {
                        // 1. 获得此时站点到world的位姿 bTt
                        boost::shared_ptr<geometry_msgs::PoseStamped const> model_posestamped = 
                                    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/obj_to_robot_holdon", this->nh_, ros::Duration(5.0));
                        geometry_msgs::Pose mesh_pose;
                        if (model_posestamped != nullptr) {
                            ROS_INFO("Received PoseStamped message:");
                            ROS_INFO("Position - x: %f, y: %f, z: %f", model_posestamped->pose.position.x, 
                                                                    model_posestamped->pose.position.y, 
                                                                    model_posestamped->pose.position.z);
                            ROS_INFO("Orientation - x: %f, y: %f, z: %f, w: %f", 
                                                                    model_posestamped->pose.orientation.x, 
                                                                    model_posestamped->pose.orientation.y, 
                                                                    model_posestamped->pose.orientation.z, 
                                                                    model_posestamped->pose.orientation.w);
                            mesh_pose = model_posestamped->pose;
                        }else{
                            ROS_INFO("Failed to receive PoseStamped message.");
                            return "error";
                        }

                        // 2. 获得站点内位姿的数据
                        movename = matches[7].str() + "_station";
                        geometry_msgs::Pose pose;
                        if(!Record_tool::get_rel_pose(movename, rel_pose_, pose))
                            return "error";
                        
                        // 3. 获得站内位姿在world内——位姿
                        tf2::Transform T_pose, T_mesh_pose;
                        tf2::fromMsg(pose, T_pose);
                        tf2::fromMsg(mesh_pose, T_mesh_pose);

                        tf2::Transform T_Ltool_world = T_mesh_pose * T_pose;
                        geometry_msgs::Pose tool_world_pose;
                        tool_world_pose.position.x = T_Ltool_world.getOrigin().x();
                        tool_world_pose.position.y = T_Ltool_world.getOrigin().y();
                        tool_world_pose.position.z = T_Ltool_world.getOrigin().z();
                        tool_world_pose.orientation.x = T_Ltool_world.getRotation().x();
                        tool_world_pose.orientation.y = T_Ltool_world.getRotation().y();
                        tool_world_pose.orientation.z = T_Ltool_world.getRotation().z();
                        tool_world_pose.orientation.w = T_Ltool_world.getRotation().w();

                        if(arm_robot_ptr->move_targetPose(tool_world_pose))
                            return "success";
                    }
                }
                else
                {
                    if(echo)std::cout << "Left Arm: No match found." << std::endl;
                    return "error";
                }
            }return "error";
        }
        // 机械臂运动，点位名称或姿态点(x,y,z,r,p,y)传值
        else if (first_string == "a")
        {
            input=input.substr(2);
            std::string temp,switch_robot,planner,poseJP;
            if(temp=input[0],temp=="D"||temp=="L"||temp=="R"||temp=="S"){
                std::cout << "Goal Robot: " << temp << std::endl;
                switch_robot=temp;
                input=input.substr(2);
            }else
            {
                ROS_WARN("Robot: No match found.");
                return "error";
            }

            if(temp=input.substr(0,3),temp=="RRT"||temp=="PTP"||temp=="LIN"){
                std::cout << "Planer Name: " << temp << std::endl;
                planner=temp;
                input=input.substr(4);
            }else{
                ROS_WARN("Planner: No match found.");
                return "error";
            }

            if(temp=input.substr(0,1),temp=="J"||temp=="P"){
                std::cout << "Pose J/P: " << temp << std::endl;
                poseJP=temp;
                input=input.substr(2);
            }else{
                ROS_WARN("Pose J/P: No match found.");
                return "error";
            }

            std::stringstream ss(input);
            float Move_vel,Move_acc;
            std::string Lname,Rname;
            float Lx,Ly,Lz,LR,LP,LY;
            float Rx,Ry,Rz,RR,RP,RY;
            ss>>Move_vel>>Move_acc;
            if(ss.fail()){
                ROS_WARN("Move vel and acc not found.");
                return "error";
            }
            std::cout<<"Move vel:"<<Move_vel<<"  acc:"<<Move_acc<<std::endl;
            std::getline(ss>>std::ws,input);
            ss=std::stringstream(input);
            // ss>>Lname;
            // std::cout<<"first get:"<<Lname<<std::endl;
            if(switch_robot=="D"){
                ss>>Lx>>Ly>>Lz>>LR>>LP>>LY>>Rx>>Ry>>Rz>>RR>>RP>>RY;
                if(ss.fail()){
                    ss=std::stringstream(input);
                    ss>>Lx>>Ly>>Lz>>LR>>LP>>LY>>Rname;
                    if(ss.fail()){
                        ss=std::stringstream(input);
                        ss>>Lname>>Rx>>Ry>>Rz>>RR>>RP>>RY;
                        if(ss.fail()){
                            ss=std::stringstream(input);
                            ss>>Lname>>Rname;
                            if(ss.fail()){
                                ROS_WARN("Pose and Joint not found.");
                                return "error";
                            }
                        }
                    }
                }
            }else if(switch_robot=="L"||switch_robot=="R"){
                ss>>Lx>>Ly>>Lz>>LR>>LP>>LY;
                if(ss.fail()){
                    printf("failed4\n");
                    ss=std::stringstream(input);
                    ss>>Lname;
                    std::cout<<"temp got:"<<Lname<<std::endl;
                    if(ss.fail()){
                        ROS_WARN("Pose and Joint not found.");
                        return "error";
                    }
                }
                if(switch_robot=="R"){
                    Rx=Lx;
                    Ry=Ly;
                    Rz=Lz;
                    RR=LR;
                    RP=LP;
                    RY=LY;
                    Rname=Lname;
                    Lname="";
                }
            }
            moveit::planning_interface::MoveGroupInterfacePtr robot_ptr;
            if(switch_robot=="D"||switch_robot=="S")robot_ptr=move_group_ptr;
            else if(switch_robot=="L")robot_ptr=left_mgtr;
            else if(switch_robot=="R")robot_ptr=right_mgtr;

            if (planner== "RRT")
            {
                robot_ptr->setPlanningPipelineId("ompl");
                robot_ptr->setPlannerId("RRT");
            }
            else if (planner== "PTP")
            {
                robot_ptr->setPlanningPipelineId("pilz_industrial_motion_planner");
                robot_ptr->setPlannerId("PTP");
            }
            else if (planner== "LIN")
            {
                if(switch_robot=="D"){
                    ROS_WARN("Dual arm can't use LIN planner!!!");
                    return "error";
                }
                robot_ptr->setPlanningPipelineId("pilz_industrial_motion_planner");
                robot_ptr->setPlannerId("LIN");
            }else{
                ROS_WARN("No planner %s found!!!",planner.c_str());
                return "error";
            }
            robot_ptr->setMaxVelocityScalingFactor(Move_vel);
            robot_ptr->setMaxAccelerationScalingFactor(Move_acc);
            
            if(poseJP=="J"){
                if(switch_robot=="D"&&(Lname==""||Rname=="")){
                    ROS_WARN("Joint and Pose conflict");
                    return "error";
                }
                if(switch_robot=="L"&&Lname==""){
                    ROS_WARN("Joint and Pose conflict");
                    return "error";
                }
                if(switch_robot=="R"&&Rname==""){
                    ROS_WARN("Joint and Pose conflict");
                    return "error";
                }
                if(switch_robot=="S"&&Lname==""){
                    ROS_WARN("Joint and Pose conflict");
                    return "error";
                }
            }
            geometry_msgs::Pose Lpose, Rpose;
            std::vector<double> Ljoints, Rjoints;
            if(Lname!=""){
                if(poseJP=="P"){
                    if(switch_robot!="S")Lname="left_"+Lname;
                    if(!Record_tool::get_abs_pose(Lname, abs_pose_, Lpose))
                        return "error";
                    geometry_msgs::Point P=Lpose.position;
                    geometry_msgs::Quaternion Q=Lpose.orientation;
                    ROS_INFO("Left Robot Pose x=%6.4f y=%6.4f z=%6.4f",P.x,P.y,P.z);
                    ROS_INFO("Left Robot Quat qx=%6.4f qy=%6.4f qz=%6.4f qw=%6.4f",Q.x,Q.y,Q.z,Q.w);
                    quaternionToRPY(Q,"Left");
                }else{
                    if(switch_robot=="S")Lname=Lname+"_inverse";
                    else Lname="left_"+Lname+"_inverse";
                    if(!Record_tool::get_joint(Lname, joints_, Ljoints))
                        return "error";
                }
                // printf("name:%s\n",Lname.c_str());
            }else if(switch_robot!="R"){
                ROS_INFO("Left Robot Pose x=%6.4f y=%6.4f z=%6.4f",Lx,Ly,Lz);
                ROS_INFO("Left Robot RPY r=%6.2f p=%6.2f y=%6.2f",LR,LP,LY);
                xyzrpyToPose(Lx,Ly,Lz,LR,LP,LY,Lpose,"Left");
                geometry_msgs::Point P=Lpose.position;
                geometry_msgs::Quaternion Q=Lpose.orientation;
            }
            if(Rname!=""){
                if(poseJP=="P"){
                    Rname="right_"+Rname;
                    if(!Record_tool::get_abs_pose(Rname, abs_pose_,Rpose))
                        return "error";
                    geometry_msgs::Point P=Rpose.position;
                    geometry_msgs::Quaternion Q=Rpose.orientation;
                    ROS_INFO("Right Robot Pose x=%6.4f y=%6.4f z=%6.4f",P.x,P.y,P.z);
                    ROS_INFO("Right Robot Quat qx=%6.4f qy=%6.4f qz=%6.4f qw=%6.4f",Q.x,Q.y,Q.z,Q.w);
                    quaternionToRPY(Q,"Right");
                }else{
                    Rname="right_"+Rname+"_inverse";
                    if(!Record_tool::get_joint(Rname, joints_,Rjoints))
                        return "error";
                }
                // printf("name:%s\n",Rname.c_str());
            }else if(switch_robot!="L"){
                ROS_INFO("Right Robot Pose x=%6.4f y=%6.4f z=%6.4f",Lx,Ly,Lz);
                ROS_INFO("Right Robot RPY r=%6.2f p=%6.2f y=%6.2f",LR,LP,LY);
                xyzrpyToPose(Rx,Ry,Rz,RR,RP,RY,Rpose,"Right");
            }

            if(switch_robot=="D"){
                if(poseJP=="P"&&dual_robot_ptr->move_targetPose(Lpose,Rpose,true,echo))
                    return "success";
                else if(poseJP=="J"&&dual_robot_ptr->move_targetJoints(Ljoints,Rjoints,true,echo))
                    return "success";
            }else if(switch_robot=="L"){
                if(poseJP=="P"&&left_robot_ptr->move_targetPose(Lpose,true,echo))
                    return "success";
                else if(poseJP=="J"&&left_robot_ptr->move_targetJoints(Ljoints,true,echo))
                    return "success";
            }else if(switch_robot=="R"){
                if(poseJP=="P"&&right_robot_ptr->move_targetPose(Rpose,true,echo))
                    return "success";
                else if(poseJP=="J"&&right_robot_ptr->move_targetJoints(Rjoints,true,echo))
                    return "success";
            }else if(switch_robot=="S"){
                if(poseJP=="P"&&arm_robot_ptr->move_targetPose(Lpose,true,echo))
                    return "success";
                else if(poseJP=="J"&&arm_robot_ptr->move_targetJoints(Ljoints,true,echo))
                    return "success";
            }return "error";
        }
        // 获取机械臂关节角和姿态值（仅返回关节角数值）
        else if (first_string == "J" || first_string == "P")
        {
            // 正则表达式：匹配机械臂标识（D/L/R）
            std::regex second_pattern("^\\S+\\s+(\\S+)");
            std::string second_string;
            std::string result = "error";  // 默认返回错误

            // 提取机械臂标识参数
            if (std::regex_search(input, matches, second_pattern))
            {
                if(echo) std::cout << "目标机械臂: " << matches[1] << std::endl;
                second_string = matches[1];
            }
            else
            {
                if(echo) std::cout << "未找到匹配的机械臂标识" << std::endl;
                return "error: no robot identifier";
            }

            try
            {
                // 处理单臂情况（L/R）
                if (second_string == "L" || second_string == "R" || second_string == "S")
                {
                    auto robot_ptr = (second_string=="S")?move_group_ptr:((second_string == "L") ? left_mgtr : right_mgtr);
                    
                    // 获取关节角 (J指令) - 仅返回6个浮点数，用逗号分隔
                    if (first_string == "J")
                    {
                        std::vector<double> joints = robot_ptr->getCurrentJointValues();
                        // 确保关节角数量为6个（机械臂通常有6个关节）
                        if (joints.size() >= 6)
                        {
                            result = std::to_string(joints[0]);
                            for (int i = 1; i < 6; ++i)
                            {
                                result += "," + std::to_string(joints[i]);
                            }
                        }
                        else
                        {
                            return "error: invalid joint joint count";
                        }
                    }
                    // 获取姿态值 (P指令) - 仅返回位置和姿态的7个浮点数
                    else if (first_string == "P")
                    {
                        geometry_msgs::Pose current_pose = robot_ptr->getCurrentPose().pose;
                        // std::string reference_frame = robot_ptr->getPlanningFrame();
                        // ROS_INFO("MoveGroup 参考坐标系（Reference Frame）: %s", reference_frame.c_str());
                        // std::string tool_frame = robot_ptr->getEndEffectorLink();
                        // ROS_INFO("MoveGroup 工具坐标系（Tool Frame）: %s", tool_frame.c_str());

                        result = std::to_string(current_pose.position.x)
                            + "," + std::to_string(current_pose.position.y)
                            + "," + std::to_string(current_pose.position.z)
                            + "," + std::to_string(current_pose.orientation.x)
                            + "," + std::to_string(current_pose.orientation.y)
                            + "," + std::to_string(current_pose.orientation.z)
                            + "," + std::to_string(current_pose.orientation.w);
                    }
                }
                // 处理双臂情况（D）- 分别返回左右臂的6个关节角，共12个浮点数
                else if (second_string == "D")
                {
                    // 获取关节角 (J指令) - 仅返回12个浮点数（左臂6个+右臂6个）
                    if (first_string == "J")
                    {
                        std::vector<double> left_joints = left_mgtr->getCurrentJointValues();
                        std::vector<double> right_joints = right_mgtr->getCurrentJointValues();
                        
                        // 确保左右臂都有至少6个关节角
                        if (left_joints.size() >= 6 && right_joints.size() >= 6)
                        {
                            // 左臂6个关节角
                            result = std::to_string(left_joints[0]);
                            for (int i = 1; i < 6; ++i)
                            {
                                result += "," + std::to_string(left_joints[i]);
                            }
                            // 右臂6个关节角
                            for (int i = 0; i < 6; ++i)
                            {
                                result += "," + std::to_string(right_joints[i]);
                            }
                        }
                        else
                        {
                            return "error: invalid joint count for dual arm";
                        }
                    }
                    // 获取姿态值 (P指令) - 仅返回14个浮点数（左臂7个+右臂7个）
                    else if (first_string == "P")
                    {
                        geometry_msgs::Pose left_pose = left_mgtr->getCurrentPose().pose;
                        geometry_msgs::Pose right_pose = right_mgtr->getCurrentPose().pose;
                        
                        result = std::to_string(left_pose.position.x)
                            + "," + std::to_string(left_pose.position.y)
                            + "," + std::to_string(left_pose.position.z)
                            + "," + std::to_string(left_pose.orientation.x)
                            + "," + std::to_string(left_pose.orientation.y)
                            + "," + std::to_string(left_pose.orientation.z)
                            + "," + std::to_string(left_pose.orientation.w)
                            + "," + std::to_string(right_pose.position.x)
                            + "," + std::to_string(right_pose.position.y)
                            + "," + std::to_string(right_pose.position.z)
                            + "," + std::to_string(right_pose.orientation.x)
                            + "," + std::to_string(right_pose.orientation.y)
                            + "," + std::to_string(right_pose.orientation.z)
                            + "," + std::to_string(right_pose.orientation.w);
                    }
                }
                else
                {
                    return "error: invalid robot identifier (" + second_string + ")";
                }
            }
            catch (const std::exception& e)
            {
                if(echo) std::cout << "获取数据失败: " << e.what() << std::endl;
                return "error: " + std::string(e.what());
            }

            return result;
        }
        // 重定位，参数：站点名称、机械臂标识: "R L dryer"
        if (first_string == "R") {
            // 检查重定位服务是否启用
            if (!Locator_flag) {
                if (echo) {
                    std::cout << "launch文件中未开启重定位服务（Locator_flag=false）" << std::endl;
                }
                return "error: locator service not initialized";
            }

            // 正则匹配："指令 机械臂 站点"格式（例如"R L uv"）
            std::regex second_pattern("^\\S+\\s+(\\S+)\\s+(\\S+)");
            std::string station, arm;
            std::string result = "error: unknown";

            if (std::regex_search(input, matches, second_pattern)) {
                // 解析机械臂标识和站点名称
                arm = matches[1];
                if(arm == "L")arm="left";
                else if(arm == "R")arm="right";
                else if(arm == "S")arm="arm";
                else{
                    result = "error: invalid arm identifier (" + arm + ")";
                    std::cout << "机械臂标识错误，仅支持L（左臂）或R（右臂）或S（单臂）" << std::endl;
                    ROS_WARN("%s", result.c_str());
                    return result;
                }
                station = matches[2];
                
                if (echo) {
                    std::cout << "机械臂: " << arm << " 重定位站点: " << station << std::endl;
                }

                // 构造JSON格式的服务请求命令
                std::stringstream ss;
                ss << "{"
                << "\"station\": \"" << station << "\","
                << "\"arm\": \"" << arm << "\","
                << "\"task\": \"multiloc\""
                << "}";
                std::string command_json = ss.str();

                // 调用定位服务
                locatornew::Location srv;
                srv.request.command_json = command_json;
                
                if (locator_client.call(srv)) {
                    // 服务调用成功，解析响应结果
                    std::string service_status = srv.response.status;
                    std::vector<float> bTt = srv.response.cached_bTt;
                    std::string result_station = srv.response.cached_station;

                    if (srv.response.result_valid) {
                        // 验证站点一致性
                        if (result_station == station) {
                            result = "success: relocation completed. ";
                            
                            // 解析bTt矩阵的位置信息（4x4矩阵展平后的第4、8、12元素）
                            if (bTt.size() == 16) {
                                result += "bTt位置: (" 
                                        + std::to_string(bTt[3]) + ", "   // x坐标
                                        + std::to_string(bTt[7]) + ", "   // y坐标
                                        + std::to_string(bTt[11]) + ")";  // z坐标
                            } else {
                                result += "warning: bTt matrix format invalid";
                            }
                            ROS_INFO("%s", result.c_str());
                        } else {
                            result = "error: station mismatch (" + result_station + " != " + station + ")";
                            ROS_WARN("%s", result.c_str());
                        }
                    } else {
                        result = "error: relocation failed - " + service_status;
                        ROS_ERROR("%s", result.c_str());
                    }
                } else {
                    // 服务调用失败
                    result = "error: failed to call locator service";
                    if (echo) {
                        std::cerr << "服务调用失败：无法连接到/locator_service" << std::endl;
                    }
                    ROS_ERROR("%s", result.c_str());
                }
            } else {
                // 正则匹配失败
                result = "error: invalid format (expected 'R <arm> <station>')";
                if (echo) {
                    std::cout << "未找到匹配的站点或机械臂标识，格式应为：R [机械臂] [站点]" << std::endl;
                }
                ROS_WARN("%s", result.c_str());
            }

            return result;
        }
    
        // 无参函数调用
        else if (first_string == "O")
        {
            std::regex Oldmove_pattern("^([a-zA-Z])\\s+([a-zA-Z0-9_]+)$");
            if (std::regex_search(input, matches, Oldmove_pattern))
            {
                if(echo)std::cout << "Move pattern: " << matches[2] << std::endl;
                return code_station_func(matches[2].str());
            }
            else
            {
                if(echo)std::cout << "Old move: No match found." << std::endl;
                return "error";
            }
        }
        // 含不定参数函数调用
        else if (first_string == "o")
        {
            std::regex Oldmove_pattern("^([a-zA-Z])\\s+([a-zA-Z0-9_]+)(.*)$");
            if (std::regex_search(input, matches, Oldmove_pattern)) {
                std::string operation = matches[2].str();
                std::string arg_str = matches[3].str();
                std::vector<std::any> args;
                // 简单解析参数，可根据实际情况修改
                size_t pos = 0;
                while ((pos = arg_str.find(' ')) != std::string::npos) {
                    std::string arg = arg_str.substr(0, pos);
                    // 检查是否为纯空格
                    if (!arg.empty() && arg.find_first_not_of(" \t") != std::string::npos) {
                        try {
                            double val = std::stod(arg);
                            args.emplace_back(val);
                            if(echo)ROS_INFO("get double %lf",val);
                        } catch (const std::invalid_argument&) {
                            args.emplace_back(arg);
                            if(echo)ROS_INFO("get string %s",arg.c_str());
                        }
                    }
                    arg_str.erase(0, pos + 1);
                }
                if (!arg_str.empty() && arg_str.find_first_not_of(" \t") != std::string::npos) {
                    try {
                        double val = std::stod(arg_str);
                        args.emplace_back(val);
                        if(echo)ROS_INFO("get double %lf",val);
                    } catch (const std::invalid_argument&) {
                        args.emplace_back(arg_str);
                        if(echo)ROS_INFO("get string %s",arg_str.c_str());
                    }
                }
                return code_station_func_with_args(operation, args);
            } else {
                if(echo)std::cout << "Function with args: No match found." << std::endl;
                return "error";
            }
        }
        // 延迟、人工判定
        else if (first_string == "W")//延迟，整数或浮点数，正数为定时等待，负数为终端继续判定
        {
            std::regex Wait_pattern("^([a-zA-Z])\\s+([-+]?\\d*\\.?\\d+)$");
            if (std::regex_search(input, matches, Wait_pattern))
            {
                if(echo)std::cout << "Delayed time: " << matches[2] << std::endl;
                float Delayed_time = std::stof(matches[2].str());
                if(Delayed_time<0)Operate_tool::interrupt_judge(true);
                else ros::Duration(Delayed_time).sleep();
            }
            else
            {
                if(echo)std::cout << "Wait operation: No match found." << std::endl;
            }return "success";
        }
        // 加载JP动作点数据文件、meshes
        else if (first_string == "F")
        {
            std::regex second_pattern("^\\S+\\s+(\\S+)");
            std::string second_string;
            if (std::regex_search(input, matches, second_pattern))
            {
                if(echo)std::cout << "File Type: " << matches[1] << std::endl;
                second_string = matches[1];
            }
            else
            {
                if(echo)std::cout << "Num: No match found." << std::endl;
            }

            if (second_string == "J")
            {
                std::regex File_pattern("^([a-zA-Z])\\s+([A-Z])\\s+([a-zA-Z0-9_]+)$");
                if (std::regex_search(input, matches, File_pattern))
                {
                    if(echo)std::cout << "Load file: " << matches[3] << std::endl;

                    // 将数据清零，以便重新赋值
                    this->joints_.clear();
                    this->abs_pose_.clear();
                    this->station_id_.clear();
                    this->rel_pose_.clear();
                    std::string Load_JPfile_name = matches[3].str();
                    if(Dual_arm_flag){
                        Record_tool::load_joint_pose(left_station_name="left_" + Load_JPfile_name + "_JP",
                                                    this->joints_, this->abs_pose_, this->rel_pose_, this->station_id_);
                        Record_tool::load_joint_pose(right_station_name="right_" + Load_JPfile_name + "_JP",
                                                    this->joints_, this->abs_pose_, this->rel_pose_, this->station_id_);
                    }else{
                        Record_tool::load_joint_pose(right_station_name=Load_JPfile_name + "_JP",
                                                    this->joints_, this->abs_pose_, this->rel_pose_, this->station_id_);
                    }
                    if(echo)std::cout << Load_JPfile_name << " has reloaded ." << std::endl;

                    return "success";
                }
                else
                {
                    if(echo)std::cout << "Reload file: No match found." << std::endl;
                }
                return "error";
            }
            else if (second_string == "M")
            {
                std::regex Model_pattern("^([A-Z])\\s+([A-Z])\\s+([a-zA-Z0-9_]+)\\s+([A-Z])$");
                if (std::regex_search(input, matches, Model_pattern))
                {
                    std::cout << "Load Model: " << matches[3] << std::endl;
                    std::cout << "Model Handle: " << matches[4] << std::endl;

                    std::string Load_Model_name = matches[3].str();

                    std::string object_name = Load_Model_name;
                    if (matches[4] == "A")
                    {
                        boost::shared_ptr<std_msgs::String const> model_index = 
                               ros::topic::waitForMessage<std_msgs::String>("/class_order_holdon", this->nh_, ros::Duration(5.0));
                        std::string Load_object_name;
                        if (model_index != nullptr) {
                            ROS_INFO("Received class_order: %d", model_index->data);
                            Load_object_name = model_index->data;

                            std::string package_path = ros::package::getPath("manipulator");
                            std::string object_address = package_path + 
                                                        "/collision_objects/" + Load_object_name +".stl";
                            boost::shared_ptr<geometry_msgs::PoseStamped const> model_posestamped = 
                                   ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/obj_to_robot_holdon", this->nh_, ros::Duration(5.0));
                            if (model_posestamped != nullptr) {
                                ROS_INFO("Received PoseStamped message:");
                                ROS_INFO("Position - x: %f, y: %f, z: %f", model_posestamped->pose.position.x, 
                                                                           model_posestamped->pose.position.y, 
                                                                           model_posestamped->pose.position.z);
                                ROS_INFO("Orientation - x: %f, y: %f, z: %f, w: %f", 
                                                                           model_posestamped->pose.orientation.x, 
                                                                           model_posestamped->pose.orientation.y, 
                                                                           model_posestamped->pose.orientation.z, 
                                                                           model_posestamped->pose.orientation.w);
                                geometry_msgs::Pose mesh_pose = model_posestamped->pose;
                                dual_robot_ptr->addobject_mesh(object_name, object_address, mesh_pose);
                            } else {
                                ROS_ERROR("Failed to receive /obj_to_robot_holdon message");
                                return "error";
                            }
                        } else {
                            ROS_ERROR("Failed to receive /class_order message");
                            return "error";
                        }
                    }
                    else if (matches[4] == "D")
                    {
                        dual_robot_ptr->removeobject(object_name);
                    }
                }
                else
                {
                    std::cout << "Load Model: No match found." << std::endl;
                }
            }
        }return "error";
    }
}