#include "manipulator/headfile.h"
#include "manipulator/robot_arm.h"
#include "std_msgs/String.h"
void record_robot_PJ(ros::NodeHandle nh);
bool record_robot_PJ_part(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                          std::string writefile_name, std::string name);

void record_station_PJ(ros::NodeHandle nh);
void record_station_PJ_part(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                          std::string writefile_name, std::string name);
geometry_msgs::Pose compute_RelPose(geometry_msgs::Pose stationToworld, 
                                    geometry_msgs::Pose toolToworld);
void convert_station_PJ(ros::NodeHandle nh);

std::string station_name="fake_station";
bool has_received = false;
void classOrderCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data!=station_name){
        ROS_INFO("站点名称更新为: %s", msg->data.c_str());
        station_name = msg->data;
        has_received = true;
    }
}
int main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "robot_record");
    ros::NodeHandle nh;

    // 开启多线程
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber sub = nh.subscribe("/class_order_holdon", 10, classOrderCallback);
    unsigned int Record_choose_flag = 1;
    while(true){
        ////////////////接下来，根据选择，实现不同的功能/////////////////////////////
        std::cout << "------请选择输入:------\n"
                  << "输入0: 机械臂点位记录 \n"
                  << "输入1: 站点点位记录 \n"
                  << "输入2: 绝对位姿转化站点相对位姿(!!!!!!!!!慎重选择，会覆盖所有目标站点的相对位姿数据!!!!!!!) \n " << std::endl;
        std::cin >> Record_choose_flag;
    
        switch (Record_choose_flag)
        {
        case 0:
            record_robot_PJ(nh);
            break;
        case 1:
            record_station_PJ(nh);
            break;
        case 2:
            convert_station_PJ(nh);
            break;
        default:
            ROS_WARN("There is no specific processing for this selection!");
            return 0;
        }
    }

    return 0;
}

void record_robot_PJ(ros::NodeHandle nh)
{
    std::string Record_JPfile_name = "gelrobot";
    if (nh.getParam("B_record/Record_JPfile", Record_JPfile_name))
        ROS_INFO("Record JPfile name: %s", Record_JPfile_name.c_str());
    else
        ROS_ERROR("Failed to find JPfile 'Record_JPfile'");

    ros::Duration delay_duration(1.0);

    std::cout << "------请选择输入要记录的规划组名称\n输入0: dual_robot \n输入1: left_robot \n输入2: right_robot \n输入3: single_robot\n"
              << std::endl;
    unsigned int Group_choose_flag = 0;
    std::cin >> Group_choose_flag;

    moveit::planning_interface::MoveGroupInterfacePtr left_move_group_ptr, right_move_group_ptr, move_group_ptr;
    std::string left_writefile_name, right_writefile_name, writefile_name;
    switch (Group_choose_flag)
    {
    case 0:
        left_writefile_name = "left_" + Record_JPfile_name + "_JP";
        right_writefile_name = "right_" + Record_JPfile_name + "_JP";
        left_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_robot");
        right_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_robot");

        std::cout << "若需退出，请按'N',继续记录点位请按'Y'" << std::endl;
        while (Operate_tool::interrupt_judge())
        {
            std::cout << "请输入你要保存点位的名称" << std::endl;
            std::string name;
            std::cin >> name;
            std::string left_real_name = "left_" + name;
            std::string right_real_name = "right_" + name;

            record_robot_PJ_part(left_move_group_ptr, left_writefile_name, left_real_name);
            record_robot_PJ_part(right_move_group_ptr, right_writefile_name, right_real_name);
        }

        break;
    case 1:
        left_writefile_name = "left_" + Record_JPfile_name + "_JP";
        left_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_robot");

        std::cout << "若需退出，请按'N',继续记录点位请按'Y'" << std::endl;
        while (Operate_tool::interrupt_judge())
        {
            std::cout << "请输入你要保存点位的名称" << std::endl;
            std::string name;
            std::cin >> name;
            std::string real_name = "left_" + name;
            record_robot_PJ_part(left_move_group_ptr, left_writefile_name, real_name);
        }
        break;

    case 2:
        right_writefile_name = "right_" + Record_JPfile_name + "_JP";
        right_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_robot");

        std::cout << "若需退出，请按'N',继续记录点位请按'Y'" << std::endl;
        while (Operate_tool::interrupt_judge())
        {
            std::cout << "请输入你要保存点位的名称" << std::endl;
            std::string name;
            std::cin >> name;
            std::string real_name = "right_" + name;
            record_robot_PJ_part(right_move_group_ptr, right_writefile_name, real_name);
        }
        break;

    case 3:
        writefile_name = Record_JPfile_name + "_JP";
        move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");

        std::cout << "若需退出，请按'N',继续记录点位请按'Y'" << std::endl;
        while (Operate_tool::interrupt_judge())
        {
            std::cout << "请输入你要保存点位的名称" << std::endl;
            std::string name;
            std::cin >> name;
            std::string real_name = name;
            record_robot_PJ_part(move_group_ptr, writefile_name, real_name);
        }
        break;
    default:
        ROS_WARN("Please enter the correct planning group...");
        break;
    }
}

bool record_robot_PJ_part(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                          std::string writefile_name, std::string name)
{
    geometry_msgs::Pose current_pose = mgptr->getCurrentPose().pose;
    std::vector<double> joints = mgptr->getCurrentJointValues();

    // Delay, waiting for the parameter amplitude to complete
    ros::Duration delay_duration(1.0);
    delay_duration.sleep();
    std::string file_root =
        ros::package::getPath("manipulator") + "/data/";
    for(int i=0;i<5;i++){
        ros::Duration delay_duration(0.1);
        if(!Record_tool::writeJoints2File(file_root + writefile_name + ".json", name + "_inverse", joints))
            continue;
        if(!Record_tool::writePose2File(file_root + writefile_name + ".json", name, current_pose))
            continue;
        return true;
    }
    ROS_ERROR("record robot pose and joints failed!!!");
    return false;
}
void record_station_PJ(ros::NodeHandle nh)
{
    if(station_name=="fake_station"){
        ROS_WARN("please relocate and set the station name by topic /class_order_holdon");
        return;
    }
    std::cout << "------站点<<" << station_name << ">>，请选择输入要记录的规划组名称\n输入0: dual_robot \n输入1: left_robot \n输入2: right_robot \n输入3: single_robot\n"
              << std::endl;
    unsigned int Group_choose_flag = 0;
    std::cin >> Group_choose_flag;

    moveit::planning_interface::MoveGroupInterfacePtr left_move_group_ptr, right_move_group_ptr, move_group_ptr;
    std::string left_writefile_name, right_writefile_name, writefile_name;
    switch (Group_choose_flag)
    {
    case 0:
        left_writefile_name = "left_" + station_name + "_JP";
        right_writefile_name = "right_" + station_name + "_JP";
        left_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_robot");
        right_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_robot");

        std::cout << "若需退出，请按'N',继续记录点位请按'Y'" << std::endl;
        while (Operate_tool::interrupt_judge())
        {
            std::cout << "请输入你要保存点位的名称" << std::endl;
            std::string name;
            std::cin >> name;
            std::string left_real_name = "left_" + name;
            std::string right_real_name = "right_" + name;

            record_station_PJ_part(left_move_group_ptr, left_writefile_name, left_real_name);
            record_station_PJ_part(right_move_group_ptr, right_writefile_name, right_real_name);
        }

        break;
    case 1:
        left_writefile_name = "left_" + station_name + "_JP";
        left_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_robot");

        std::cout << "若需退出，请按'N',继续记录点位请按'Y'" << std::endl;
        while (Operate_tool::interrupt_judge())
        {
            std::cout << "请输入你要保存点位的名称" << std::endl;
            std::string name;
            std::cin >> name;
            std::string real_name = "left_" + name;
            record_station_PJ_part(left_move_group_ptr, left_writefile_name, real_name);
        }
        break;

    case 2:
        right_writefile_name = "right_" + station_name + "_JP";
        right_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_robot");

        std::cout << "若需退出，请按'N',继续记录点位请按'Y'" << std::endl;
        while (Operate_tool::interrupt_judge())
        {
            std::cout << "请输入你要保存点位的名称" << std::endl;
            std::string name;
            std::cin >> name;
            std::string real_name = "right_" + name;
            record_station_PJ_part(right_move_group_ptr, right_writefile_name, real_name);
        }
        break;

    case 3:
        writefile_name = station_name + "_JP";
        move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");

        std::cout << "若需退出，请按'N',继续记录点位请按'Y'" << std::endl;
        while (Operate_tool::interrupt_judge())
        {
            std::cout << "请输入你要保存点位的名称" << std::endl;
            std::string name;
            std::cin >> name;
            std::string real_name = "" + name;
            record_station_PJ_part(move_group_ptr, writefile_name, real_name);
        }
        break;
    default:
        ROS_WARN("Please enter the correct planning group...");
        break;
    }
}

void record_station_PJ_part(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                            std::string writefile_name, std::string name)
{
    ros::NodeHandle nh;
    geometry_msgs::Pose current_pose = mgptr->getCurrentPose().pose;
    boost::shared_ptr<geometry_msgs::PoseStamped const> model_posestamped =
        ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/obj_to_robot_holdon", nh, ros::Duration(5.0));
    if (model_posestamped != nullptr)
    {
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
    }
    geometry_msgs::Pose station_pose;
    station_pose = Record_tool::computeRelPose(model_posestamped->pose, current_pose);

    printf("current %s (%.4lf,%.4lf,%.4lf)->(%.4lf,%.4lf,%.4lf)",name.c_str(),
                    current_pose.position.x,current_pose.position.y,current_pose.position.z,
                    station_pose.position.x,station_pose.position.y,station_pose.position.z);


    std::vector<double> joints = mgptr->getCurrentJointValues();

    // Delay, waiting for the parameter amplitude to complete
    ros::Duration delay_duration(1.0);
    delay_duration.sleep();
    std::string file_root =
        ros::package::getPath("manipulator") + "/data/";
    for (int i = 0; i < 3; i++)
    {
        if(!Record_tool::writeJoints2File(file_root + writefile_name + ".json", name + "_inverse", joints))
            continue;
        if(!Record_tool::writePose2File(file_root + writefile_name + ".json", name, current_pose))
            continue;
        if(!Record_tool::writeStation2File(file_root + writefile_name + ".json", name + "_station", station_pose))
            continue;
        ros::Duration delay_duration(0.1);
    }
}

void convert_station_PJ(ros::NodeHandle nh)
{
    std::string left_writefile_name = "left_" + station_name + "_JP";
    std::string right_writefile_name = "right_" + station_name + "_JP";

    boost::shared_ptr<geometry_msgs::PoseStamped const> model_posestamped =
        ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/obj_to_robot_holdon", nh, ros::Duration(5.0));
    if (model_posestamped != nullptr)
    {
        ROS_INFO("Received PoseStamped message:");
        ROS_INFO("Position - x: %f, y: %f, z: %f", model_posestamped->pose.position.x,
                 model_posestamped->pose.position.y,
                 model_posestamped->pose.position.z);
        ROS_INFO("Orientation - x: %f, y: %f, z: %f, w: %f",
                 model_posestamped->pose.orientation.x,
                 model_posestamped->pose.orientation.y,
                 model_posestamped->pose.orientation.z,
                 model_posestamped->pose.orientation.w);
    }
    
    std::string file_root =
        ros::package::getPath("manipulator") + "/data/";
    Record_tool::convertPose2Station(file_root+left_writefile_name+".json",model_posestamped->pose);
    Record_tool::convertPose2Station(file_root+right_writefile_name+".json",model_posestamped->pose);
    std::cout<<"成功转换绝对位姿至站点相对位姿"<<std::endl;
}
