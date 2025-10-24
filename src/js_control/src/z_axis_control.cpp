#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager_msgs/SwitchController.h>

#include <csignal>

using namespace std;

void handleCtrlC(int signal)
{
    controller_manager_msgs::SwitchController switchRequest;
    ros::ServiceClient switchClient;
    ros::NodeHandle nh;
    switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_js");
    ros::NodeHandle nh;

    signal(SIGINT, handleCtrlC);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::ServiceClient _srv;
    _srv = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    _srv.waitForExistence();
    controller_manager_msgs::SwitchController sw;
    sw.request.start_controllers = vector<string>{"twist_controller"};
    sw.request.stop_controllers = vector<string>{"scaled_pos_joint_traj_controller"};
    sw.request.strictness = sw.request.BEST_EFFORT;
    _srv.call(sw);
    if (!sw.response.ok)
    {
        ROS_ERROR("Cannot swtich controller");
        ros::shutdown();
    }
    std::cout << "switched to twist_controller" << std::endl;

    auto rate = ros::Rate(2);
    unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> realtime_pub;

    realtime_pub = make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, "/twist_controller/command", 4);

    float scale_tra_angle = 4 * 3.14 / 50;
    float scale_rot_angle = 4 * 3.14 / 50;
    unsigned int count = 0;
    int inverse_flag = 1;
    bool setup_flag = 0;
    bool setup_flag_new = 0;
    bool new_flag = 1;

    while (ros::ok())
    {

        if (realtime_pub->trylock())
        {
            if (new_flag == 1)
            {
                realtime_pub->msg_.angular.x = 2*scale_rot_angle;
                ROS_INFO("sending message(x)......");
                new_flag = 0;
                realtime_pub->unlockAndPublish();
            }
            else
            {
                realtime_pub->msg_.angular.x = 0;

                realtime_pub->msg_.angular.z = inverse_flag * scale_rot_angle;
                ROS_INFO("sending message......");
                count += inverse_flag;
                realtime_pub->unlockAndPublish();
                setup_flag = 1;
            }
        }

        if (count == 25 && setup_flag)
        {
            realtime_pub->msg_.angular.z = 0;
            inverse_flag = -1 * inverse_flag;
            setup_flag_new = 1;
            ROS_INFO("invesing......");
            rate.sleep();
            rate.sleep();
        }
        if (count == 1 && setup_flag_new)
        {
            realtime_pub->msg_.angular.z = 0;
            inverse_flag = -1 * inverse_flag;
            ROS_INFO("invesing......");
            rate.sleep();
            rate.sleep();
        }

        rate.sleep();
    }
    realtime_pub->msg_.angular.z = 0;
    ros::shutdown();

    return 0;
}