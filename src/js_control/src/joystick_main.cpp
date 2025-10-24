#include "js_control/joystick_control.h"
#include "std_msgs/Int32.h"
#include <csignal>
#include "gripper_modbus/Gripper.h"
#include <ros/ros.h>

// std::shared_ptr<pgi> left_pgi_ptr;
// 创建服务请求对象
gripper_modbus::Gripper pgi_srv,rgi_srv;
ros::ServiceClient PGI_client,RGI_client;
ros::Publisher adp1000_pub;
int switch_mode=1;
int control_mode=1;
int speed_mode=0;
bool Dual_arm_flag=true;
// 将base坐标系下的信息，映射到world坐标系下
tf::Matrix3x3 get_rotation_matrix(ros::NodeHandle nh, std::string);
void handleCtrlC(int signal);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "ur5_js");
    ros::NodeHandle nh;
    if (nh.getParam("/js_control/switch_mode", switch_mode)) {
        ROS_INFO("Successfully got the 'switch_mode' parameter: %d", switch_mode);
    } else {
        ROS_WARN("Failed to get the 'switch_mode' parameter. Using default value or check launch file.");
        // 可以在这里设置一个默认值
        switch_mode = 1;
    }
    if (nh.getParam("/js_control/control_mode", control_mode)) {
        ROS_INFO("Successfully got the 'control_mode' parameter: %d", control_mode);
    } else {
        ROS_WARN("Failed to get the 'control_mode' parameter. Using default value or check launch file.");
        // 可以在这里设置一个默认值
        control_mode = 1;
    }
    // 读取Dual_arm_flag参数
    if (nh.getParam("A_manipulator/Dual_arm_flag", Dual_arm_flag))
        ROS_INFO("Got param: %d", Dual_arm_flag);
    else
        ROS_ERROR("Failed to get param 'Dual_arm_flag'");

    adp1000_pub = nh.advertise<std_msgs::Int32>("/adp1000cmd", 10);

    ros::service::waitForService("/PGI_gripper_control");
    PGI_client = nh.serviceClient<gripper_modbus::Gripper>("/PGI_gripper_control");
    ros::service::waitForService("/RGI_gripper_control");
    RGI_client = nh.serviceClient<gripper_modbus::Gripper>("/RGI_gripper_control");

    // 设置请求参数
    pgi_srv.request.force              = -1 ;//平行力值，20～100
    pgi_srv.request.velocity           = -1 ;//平行速度，1～100
    pgi_srv.request.torque             = -1 ;//旋转力值，20～100
    pgi_srv.request.speed              = -1 ;//旋转速度，1～100
    pgi_srv.request.abs_angle          = 99999999 ;//绝对角度，-160*32768-32768～160*32678+32767
    pgi_srv.request.rel_angle          = 0 ;//相对角度，-32768～32767
    pgi_srv.request.position           = -1 ;//平行位置，0～1000
    pgi_srv.request.block_flag         = false ;//阻塞标志
    pgi_srv.request.stop_flag          = false ;//强制停止
    pgi_srv.request.reset_flag         = false ;//初始化
    pgi_srv.request.feedback           = 0b0000001 ;//反馈参数，仅反馈position
    rgi_srv=pgi_srv;
     
    signal(SIGINT, handleCtrlC);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    tf::Matrix3x3 lmatrix,rmatrix;
    if(Dual_arm_flag){
        ur5_js l_js_cmd(nh, Dual_arm_flag?"left_robot":"single", Dual_arm_flag?"left_tool0":"tool0", get_rotation_matrix(nh,Dual_arm_flag?"left_base":"base"));
        ur5_js r_js_cmd(nh, "right_robot", "right_tool0", get_rotation_matrix(nh,"right_base"));
        // l_js_cmd.add_end("left_pgi_end");
        // r_js_cmd.add_end("right_RGI_end");
        // r_js_cmd.add_end("right_spoon_end");
        auto rate = ros::Rate(50);
        if(control_mode==1){
            printf("takeControl");
            takeControl();
        }
        while (ros::ok())
        {
            if(switch_mode&1)l_js_cmd.run();
            if(switch_mode>>1&1)r_js_cmd.run();
            rate.sleep();
        }
    }else{
        ur5_js l_js_cmd(nh, "single", "tool0", get_rotation_matrix(nh,"base"));
        auto rate = ros::Rate(50);
        if(control_mode==1){
            printf("takeControl");
            takeControl();
        }
        while (ros::ok())
        {
            l_js_cmd.run();
            rate.sleep();
        }
        
    }
    return 0;
}
// ------------------------------------------------------------------------
tf::Matrix3x3 get_rotation_matrix(ros::NodeHandle nh, std::string base_name)//matrix为左右臂的基坐标系与世界坐标系的关系
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Matrix3x3 matrix;
    bool display_flag = false;
    while (nh.ok())
    {
        if (listener.canTransform(base_name, "/world", ros::Time(0)))
        {
            listener.lookupTransform(base_name, "/world", ros::Time(0), transform);
            tf::Quaternion orientation = transform.inverse().getRotation();
            matrix.setRotation(orientation);

            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            ROS_INFO("Have obtain ~~ %s ~~ Matrix, Roll: %f, Pitch: %f, Yaw: %f", base_name.c_str(), roll, pitch, yaw);
            return matrix;
        }
        else if (!display_flag)
        {
            display_flag = true;
            ROS_WARN("Wait for ~~ %s ~~listener response......",base_name.c_str());
        }
    }return matrix;
}

void handleCtrlC(int signal)
{
    controller_manager_msgs::SwitchController switchRequest;
    ros::ServiceClient switchClient;
    ros::NodeHandle nh;

    std::string switch_goalrobot="left_robot";
    // if (nh.getParam("/js_control/switch_goalrobot", switch_goalrobot))
    // {
    //     ROS_INFO("Have found the target robot: %s", switch_goalrobot.c_str());
    // }
    // else
    // {
    //     switch_goalrobot = "";
    //     ROS_WARN("Not found the target robot, default null");
    // }
    std::string switch_controller_name;
    switch_controller_name = switch_goalrobot + "/controller_manager/switch_controller";
    switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
    // Fill in the request with the controllers you want to start and stop
    switchRequest.request.start_controllers.push_back("scaled_pos_joint_traj_controller");
    switchRequest.request.stop_controllers.push_back("twist_controller");

    // Indicate whether you want to strict or best-effort switching
    switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

    // Call the service to switch controllers
    if (switchClient.call(switchRequest))
    {
        if (switchRequest.response.ok)
        {
            ROS_INFO("Controller switch was successful.");
        }
        else
        {
            ROS_INFO("Controller switch failed");
        }
    }
    else
    {
        ROS_ERROR("Failed to call controller switch service.");
    }

    switch_goalrobot="right_robot";
    // if (nh.getParam("/js_control/switch_goalrobot", switch_goalrobot))
    // {
    //     ROS_INFO("Have found the target robot: %s", switch_goalrobot.c_str());
    // }
    // else
    // {
    //     switch_goalrobot = "";
    //     ROS_WARN("Not found the target robot, default null");
    // }
    switch_controller_name = switch_goalrobot + "/controller_manager/switch_controller";
    switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
    // Fill in the request with the controllers you want to start and stop
    switchRequest.request.start_controllers.push_back("scaled_pos_joint_traj_controller");
    switchRequest.request.stop_controllers.push_back("twist_controller");

    // Indicate whether you want to strict or best-effort switching
    switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

    // Call the service to switch controllers
    if (switchClient.call(switchRequest))
    {
        if (switchRequest.response.ok)
        {
            ROS_INFO("Controller switch was successful.");
        }
        else
        {
            ROS_INFO("Controller switch failed");
        }
    }
    else
    {
        ROS_ERROR("Failed to call controller switch service.");
    }

    // Shutdown ROS
    ros::shutdown();
    exit(signal);
}
