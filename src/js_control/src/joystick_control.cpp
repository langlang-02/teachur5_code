#include "js_control/joystick_control.h"
#include "std_msgs/Int32.h"
#include "gripper_modbus/Gripper.h"
#include <chrono>
using namespace std;
double tra_scale_max=0.03,rot_scale_max=0.15;
double tra_scale_mini=0.005,rot_scale_mini=0.01;
double tra_scale=tra_scale_mini,rot_scale=rot_scale_mini;
// extern std::shared_ptr<pgi> left_pgi_ptr;
extern gripper_modbus::Gripper pgi_srv,rgi_srv;
extern ros::ServiceClient PGI_client,RGI_client;
extern ros::Publisher adp1000_pub;
extern int switch_mode;
extern int control_mode;
extern int speed_mode;
extern int Dual_arm_flag;
bool back_pressed = false;
bool start_pressed = false;
ros::Time back_press_start_time;
ros::Time start_press_start_time;
tf::Vector3 last_base_linear_velocity(0,0,0);
tf::Vector3 last_base_angular_velocity(0,0,0);
F710::F710()
{
    x = 0;
    a = 0;
    b = 0;
    y = 0;
    LB = 0;
    RB = 0;
    LT = 0;
    RT = 0;
    BACK = 0;
    unknown1 = 0;
    unknown2 = 0;
    unknown3 = 0;
    button_l = 0;
    button_up = 0;
    rstick_l = 0;
    rstick_up = 0;
    lstick_l = 0;
    lstick_up = 0;
}

ur5_js::ur5_js(ros::NodeHandle nh_, std::string robot_name_, std::string tool_name_, tf::Matrix3x3 world_to_base_)
{
    add_end(tool_name_);
    curr_end=end_names.begin();
    nh=nh_;
    world_to_base=world_to_base_;
    robot_name=robot_name_;
    tool_name=tool_name_;
    std::string twist_command_name, switch_controller_name;
    if(robot_name=="single"){
        twist_command_name = "/twist_controller/command";
        switch_controller_name = "/controller_manager/switch_controller";
        twist_topic_name = "/twist_param";
    }else{
        twist_command_name = "/" + robot_name + "/twist_controller/command";
        switch_controller_name = robot_name + "/controller_manager/switch_controller";
        twist_topic_name = "/" + robot_name + "/twist_param";
    }
    // robot_dbptr = std::make_shared<dashboardsrv_client>(nh, "/"+robot_name); 

    _joystick_ptr = make_unique<F710>();
    _realtime_pub = make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, twist_command_name, 4);
    _sub = nh.subscribe("/joy", 10, &ur5_js::Callback1, this);
    target_control_mode=-1;
    twist_sub = nh.subscribe(twist_topic_name, 10, &ur5_js::twistMoveCallback, this);
    _srv = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
    _srv.waitForExistence();
    // std::cout<<robot_name<<':'<<robot_name[0]<<std::endl;&&robot_name[0]=='l'
    while (!_sub.getNumPublishers())
    {
        ROS_WARN_THROTTLE(3, "waiting joystick connection");
    }
    nh.param<double>("js_control/tra_scale", _tra_scale, 0.05);
    nh.param<double>("js_control/rot_scale", _rot_scale, 0.05);
    // controller_manager_msgs::SwitchController sw;
    // sw.request.start_controllers = vector<string>{"twist_controller"};
    // sw.request.stop_controllers = vector<string>{"scaled_pos_joint_traj_controller"};s
    // sw.request.strictness = sw.request.BEST_EFFORT;
    // _srv.call(sw);
    // if (!sw.response.ok)
    // {
    //     ROS_ERROR("Cannot swtich controller");
    //     ros::shutdown();
    // }
    // std::cout << "switched to twist_controller" << std::endl;
}

void ur5_js::add_end(std::string end_name){
    end_names.push_back(end_name);
    // curr_end=end_names.end()-1;
    curr_end=end_names.begin();
}
tf::Vector3 ur5_js::get_abs_end_to_tool_vector(){
    tf::TransformListener listener;
    tf::StampedTransform end_to_world,tool_to_world;
    bool display_flag = false;
    while (nh.ok())
    {
        if (listener.canTransform("world", *curr_end, ros::Time(0)) && listener.canTransform("world", tool_name, ros::Time(0)))
        {
            listener.lookupTransform("world", *curr_end, ros::Time(0), end_to_world);
            listener.lookupTransform("world", tool_name, ros::Time(0), tool_to_world);
            tf::Transform end_to_tool = end_to_world.inverse() * tool_to_world;
            // 获取在 world 坐标系下的位置向量
            tf::Vector3 end_position = end_to_world.getOrigin();
            tf::Vector3 tool_position = tool_to_world.getOrigin();
            // 计算以 world 为参考的向量
            tf::Vector3 vector = tool_position - end_position;
            return vector;
        }
        else if (!display_flag)
        {
            display_flag = true;
            ROS_WARN("Wait for ~~ %s %s ~~listener response......",(*curr_end).c_str(),tool_name.c_str());
        }
    }return tf::Vector3();
}
ur5_js::~ur5_js()
{
    controller_manager_msgs::SwitchController sw;
    sw.request.start_controllers = vector<string>{"scaled_pos_joint_traj_controller"};
    sw.request.stop_controllers = vector<string>{"twist_controller"};
    sw.request.strictness = sw.request.BEST_EFFORT;
    _srv.call(sw);
    ros::Duration(5).sleep();
    if (!sw.response.ok)
    {
        ROS_ERROR("Cannot swtich controller");
        ros::shutdown();
    }
    std::cout << "switched to scaled_pos_joint_traj_controller" << std::endl;
}

bool RGI_handle(ros::ServiceClient client,gripper_modbus::Gripper& srv){
    // 调用服务
    if (client.call(srv)) {
        ROS_INFO("gripper status: %s",srv.response.status.c_str());
        printf("position=%d rel_angle=%d\n",srv.response.position,srv.response.abs_angle);
    } else {
        ROS_ERROR("gripper failed");
    }
    return 0;
}
void matrixPrint(std::string str,tf::Matrix3x3 matrix){
    printf("--------- %s ---------\n",str.c_str());
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            printf("%7.3f ",matrix[i][j]);
        }puts("");
    }
}
void vectorPrint(std::string str,tf::Vector3 vector){
    printf("--------- %s ---------\n",str.c_str());
    for(int i=0;i<3;i++){
        printf("%10.6f ",vector[i]);
        puts("");
    }
}
void ur5_js::run()
{
    if (_realtime_pub->trylock())
    {
        // --------------------旧----------------------------
        // // LINEAR
        // if (_joystick_ptr->x)
        //     _realtime_pub->msg_.linear.x = _tra_scale;
        // else if (_joystick_ptr->b)
        //     _realtime_pub->msg_.linear.x = -_tra_scale;
        // else
        //     _realtime_pub->msg_.linear.x = 0;

        // if (_joystick_ptr->y)
        //     _realtime_pub->msg_.linear.y = _tra_scale;
        // else if (_joystick_ptr->a)
        //     _realtime_pub->msg_.linear.y = -_tra_scale;
        // else
        //     _realtime_pub->msg_.linear.y = 0;

        // if (_joystick_ptr->LB)
        //     _realtime_pub->msg_.linear.z = _tra_scale;
        // else if (_joystick_ptr->RB)
        //     _realtime_pub->msg_.linear.z = -_tra_scale;
        // else
        //     _realtime_pub->msg_.linear.z = 0;

        // // ANGULAR
        // _realtime_pub->msg_.angular.x = _rot_scale * _joystick_ptr->lstick_up;
        // _realtime_pub->msg_.angular.y = _rot_scale * _joystick_ptr->lstick_l;
        // if (_joystick_ptr->LT)
        //     _realtime_pub->msg_.angular.z = _rot_scale;
        // else if (_joystick_ptr->RT)
        //     _realtime_pub->msg_.angular.z = -_rot_scale;
        // else
        //     _realtime_pub->msg_.angular.z = 0;
        // --------------------旧----------------------------
        // if(_joystick_ptr->x)std::cout<<"x="<<_joystick_ptr->x<<std::endl;
        // if(_joystick_ptr->y)std::cout<<"y="<<_joystick_ptr->y<<std::endl;
        // if(_joystick_ptr->a)std::cout<<"a="<<_joystick_ptr->a<<std::endl;
        // if(_joystick_ptr->b)std::cout<<"b="<<_joystick_ptr->b<<std::endl;
        // if(_joystick_ptr->LB)std::cout<<"LB="<<_joystick_ptr->LB<<std::endl;
        // if(_joystick_ptr->RB)std::cout<<"RB="<<_joystick_ptr->RB<<std::endl;
        // if(_joystick_ptr->LT)std::cout<<"LT="<<_joystick_ptr->LT<<std::endl;
        // if(_joystick_ptr->RT)std::cout<<"RT="<<_joystick_ptr->RT<<std::endl;
        // if(_joystick_ptr->BACK)std::cout<<"BACK="<<_joystick_ptr->BACK<<std::endl;
        // if(_joystick_ptr->unknown1)std::cout<<"unknown1="<<_joystick_ptr->unknown1<<std::endl;
        // if(_joystick_ptr->unknown2)std::cout<<"unknown2="<<_joystick_ptr->unknown2<<std::endl;
        // if(_joystick_ptr->unknown3)std::cout<<"unknown3="<<_joystick_ptr->unknown3<<std::endl;
        // if(_joystick_ptr->button_l)std::cout<<"button_l="<<_joystick_ptr->button_l<<std::endl;
        // if(_joystick_ptr->button_up)std::cout<<"button_up="<<_joystick_ptr->button_up<<std::endl;
        //  if(_joystick_ptr->rstick_l)std::cout<<"rstick_l="<<_joystick_ptr->rstick_l<<std::endl;
        //  if(_joystick_ptr->rstick_up)std::cout<<"rstick_up="<<_joystick_ptr->rstick_up<<std::endl;
        // if(_joystick_ptr->lstick_l)std::cout<<"lstick_l="<<_joystick_ptr->lstick_l<<std::endl;
        // if(_joystick_ptr->lstick_up)std::cout<<"lstick_up="<<_joystick_ptr->lstick_up<<std::endl;
        bool press_flag=false,press_angular_flag=false;
        if(_joystick_ptr->x)press_flag=true;
        if(_joystick_ptr->y)press_flag=true;
        if(_joystick_ptr->a)press_flag=true;
        if(_joystick_ptr->b)press_flag=true;
        if(_joystick_ptr->LB)press_flag=true;
        if(_joystick_ptr->RB)press_flag=true;
        if(_joystick_ptr->LT)press_flag=press_angular_flag=true;
        if(_joystick_ptr->RT)press_flag=press_angular_flag=true;
        // if(_joystick_ptr->BACK)press_flag=true;
        // if(_joystick_ptr->unknown1)press_flag=true;
        if(_joystick_ptr->unknown2)press_flag=true;
        if(_joystick_ptr->unknown3)press_flag=true;
        if(_joystick_ptr->button_l)press_flag=true;
        if(_joystick_ptr->button_up)press_flag=true;
        if(_joystick_ptr->rstick_l)press_flag=true;
        if(_joystick_ptr->rstick_up)press_flag=true;
        if(_joystick_ptr->lstick_l)press_flag=press_angular_flag=true;
        if(_joystick_ptr->lstick_up)press_flag=press_angular_flag=true;
        double temp_linear_x, temp_linear_y, temp_linear_z;
        double temp_angular_x, temp_angular_y, temp_angular_z;
        bool RGI_flag=false,pgi_flag=false;
        // LINEAR
        if (back_pressed && start_pressed) {
            ros::Duration press_duration_back = ros::Time::now() - back_press_start_time;
            ros::Duration press_duration_start = ros::Time::now() - start_press_start_time;
            // 假设长按时间阈值为1秒，可根据需求调整
            if (press_duration_back.toSec() >= 1.5 && press_duration_start.toSec() >= 1.5) {
                // 在这里处理同时长按的逻辑
                curr_end++;
                if(curr_end==end_names.end())
                    curr_end=end_names.begin();
                back_pressed=false;
                start_pressed=false;
                if(tra_scale!=tra_scale_mini)
                    std::cout<<"switch to min "<<tra_scale<<','<<rot_scale<<std::endl;
                tra_scale=tra_scale_mini;
                rot_scale=rot_scale_mini;
                speed_mode=0;
                ROS_INFO("switch to %s end\n",(*curr_end).c_str());
            }
        }
        if(fabs(_joystick_ptr->rstick_l)>0.1||fabs(_joystick_ptr->rstick_up)>0.1){
            if(_joystick_ptr->rstick_up<-0.9&&switch_mode!=2){
                // switch_mode=1;
                // std::cout<<"switch to left arm "<<std::endl;
                switch_mode=2;
                std::cout<<"switch to right arm "<<std::endl;
            }else if(_joystick_ptr->rstick_up>0.9&&switch_mode!=1){
                // switch_mode=2;
                // std::cout<<"switch to right arm "<<std::endl;
                switch_mode=1;
                std::cout<<"switch to left arm "<<std::endl;
            }else if(fabs(_joystick_ptr->rstick_up)<0.1&&_joystick_ptr->rstick_l>0.9&&switch_mode!=3){
                switch_mode=3;
                std::cout<<"switch to dual arm "<<std::endl;
            }else if(fabs(_joystick_ptr->rstick_up)<0.1&&_joystick_ptr->rstick_l<-0.9&&switch_mode!=7){
                switch_mode=7;
                std::cout<<"switch to opposite arm "<<std::endl;
            }
        }
        if(control_mode==1&&_joystick_ptr->unknown3==1){//右摇杆按下Rs
            freeControl();
            control_mode=0;
        }else if(control_mode==0&&_joystick_ptr->unknown2==1){//Ls press
            // robot_dbptr->unlockPS();
            // robot_dbptr->closePopUp();
            // robot_dbptr->closeSafetyPopUp();
            // robot_dbptr->robot_init();
            // robot_dbptr->load_program("pro_1.urp\n");
            // robot_dbptr->stop();
            // robot_dbptr->play();
            takeControl();
            control_mode=1;
        }else if(press_flag){
            if(control_mode)
                ROS_INFO("press Rs to free joy controller.");
            else
                ROS_INFO("press Ls to take back joy controller.");
        }
        
        if(_joystick_ptr->button_l==1){
            if(switch_mode!=2){
                pgi_srv.request.position           = -1 ;//平行位置，0～1000
                RGI_handle(PGI_client,pgi_srv);
                std::cout<<"real left PGI="<<pgi_srv.response.position<<std::endl;
                pgi_srv.request.position=max(0,pgi_srv.response.position-(speed_mode?50:5));
                std::cout<<"set left PGI="<<pgi_srv.request.position<<std::endl;
                pgi_flag=true;
            }
            if(switch_mode!=1){
                rgi_srv.request.position           = -1 ;//平行位置，0～1000
                RGI_handle(RGI_client,rgi_srv);
                std::cout<<"real right RGI="<<rgi_srv.response.position<<std::endl;
                rgi_srv.request.position=max(0,rgi_srv.response.position-(speed_mode?50:5));
                std::cout<<"set right RGI="<<rgi_srv.request.position<<std::endl;
                RGI_flag=true;
            }
        }else if(_joystick_ptr->button_l==-1){
            if(switch_mode!=2){
                pgi_srv.request.position           = -1 ;//平行位置，0～1000
                RGI_handle(PGI_client,pgi_srv);
                std::cout<<"real left PGI="<<pgi_srv.response.position<<std::endl;
                pgi_srv.request.position=min(1000,pgi_srv.response.position+(speed_mode?50:5));
                std::cout<<"set left PGI="<<pgi_srv.request.position<<std::endl;
                pgi_flag=true;
            }
            if(switch_mode!=1){
                rgi_srv.request.position           = -1 ;//平行位置，0～1000
                RGI_handle(RGI_client,rgi_srv);
                std::cout<<"real right RGI="<<rgi_srv.response.position<<std::endl;
                rgi_srv.request.position=min(1000,rgi_srv.response.position+(speed_mode?50:5));
                std::cout<<"set right RGI="<<rgi_srv.request.position<<std::endl;
                RGI_flag=true;
            }
        }

        if(switch_mode!=2){
            if(_joystick_ptr->button_up==1){//上
                pgi_srv.request.position           = -1 ;//平行位置，0～1000
                pgi_srv.request.reset_flag=true;
                RGI_handle(PGI_client,pgi_srv);
                pgi_srv.request.reset_flag=false;
                std::cout<<"reset left PGI"<<std::endl;
            }else if(_joystick_ptr->button_up==-1){//下
                rgi_srv.request.position           = -1 ;//平行位置，0～1000
                rgi_srv.request.reset_flag=true;
                RGI_handle(RGI_client,rgi_srv);
                rgi_srv.request.reset_flag=false;
                std::cout<<"reset right RGI"<<std::endl;
            }
        }else{
            if(_joystick_ptr->button_up==1){//上
                // std_msgs::Int32 ADP1000_msg;
                // ADP1000_msg.data = 0;
                // adp1000_pub.publish(ADP1000_msg);
                // std::cout << "Right ADP1000 Init"<< std::endl;
                rgi_srv.request.rel_angle=speed_mode?30:3;
                std::cout << "rotate right RGI "<<rgi_srv.request.rel_angle<<" degree"<< std::endl;
                RGI_flag=true;
            }else if(_joystick_ptr->button_up==-1){//下
                rgi_srv.request.rel_angle=-(speed_mode?30:3);
                std::cout << "rotate right RGI "<<rgi_srv.request.rel_angle<<" degree"<< std::endl;
                RGI_flag=true;
            }
        }

        if (_joystick_ptr->BACK){
            if(tra_scale!=tra_scale_max)
                std::cout<<"switch to max "<<tra_scale<<','<<rot_scale<<std::endl;
            tra_scale=tra_scale_max;
            rot_scale=rot_scale_max;
            speed_mode=1;
        }
        else if(_joystick_ptr->unknown1){
            if(tra_scale!=tra_scale_mini)
                std::cout<<"switch to min "<<tra_scale<<','<<rot_scale<<std::endl;
            tra_scale=tra_scale_mini;
            rot_scale=rot_scale_mini;
            speed_mode=0;
        }
        if (_joystick_ptr->x)
            temp_linear_x = -tra_scale;
        else if (_joystick_ptr->b)
            temp_linear_x = tra_scale;
        else
            temp_linear_x = 0;

        if (_joystick_ptr->a)
            temp_linear_y = -tra_scale;
        else if (_joystick_ptr->y)
            temp_linear_y = tra_scale;
        else
            temp_linear_y = 0;

        if (_joystick_ptr->RB)
            temp_linear_z = tra_scale;
        else if (_joystick_ptr->LB)
            temp_linear_z = -tra_scale;
        else
            temp_linear_z = 0;
        if(robot_name[0]=='r'&&switch_mode==7)temp_linear_x*=-1;
        // ANGULAR
        temp_angular_x = -rot_scale * _joystick_ptr->lstick_l;
        temp_angular_y = -rot_scale * _joystick_ptr->lstick_up;
        if (_joystick_ptr->LT)
            temp_angular_z = rot_scale;
        else if (_joystick_ptr->RT)
            temp_angular_z = -rot_scale;
        else
            temp_angular_z = 0;
        if(rot_scale==rot_scale_max)
            temp_angular_z*=5;

        // std::cout<<temp_linear_x<<" "<<temp_linear_y<<" "<<temp_linear_z<<" "<<temp_angular_x<<" "<<temp_angular_y<<" "<<temp_angular_z<<std::endl;
        // -----------------------------------------------------------
        // 假设你有世界坐标系下的线速度
        tf::Vector3 world_linear_velocity(temp_linear_x, temp_linear_y, temp_linear_z);
        tf::Vector3 world_angular_velocity(temp_angular_x, temp_angular_y, temp_angular_z);
        if(~target_control_mode){
            if(target_control_mode&&!control_mode)takeControl();
            else if(!target_control_mode&&control_mode)freeControl();
            control_mode=target_control_mode;
            world_linear_velocity=twist_world_linear_velocity;
            world_angular_velocity=twist_world_angular_velocity;
            // printf("use twist param\n");
            // vectorPrint("linear:",world_linear_velocity);
        }
        // 将线速度从末端转为工具端
        if(press_angular_flag&&curr_end!=end_names.begin()){
            // puts("~~~~~~~~~~~~~~~~~~~~~~~");
            // vectorPrint("old linear:",world_linear_velocity);
            // vectorPrint("angular:",world_angular_velocity);
            // vectorPrint(*curr_end,get_abs_end_to_tool_vector());
            world_linear_velocity+=world_angular_velocity.cross(get_abs_end_to_tool_vector());
            double magnitude = std::sqrt(
                world_linear_velocity.getX() * world_linear_velocity.getX() +
                world_linear_velocity.getY() * world_linear_velocity.getY() +
                world_linear_velocity.getZ() * world_linear_velocity.getZ()
            );
        
            if (magnitude > 0) {
                double scale_factor = tra_scale*0.7 / magnitude;
                world_linear_velocity.setX(world_linear_velocity.getX() * scale_factor);
                world_linear_velocity.setY(world_linear_velocity.getY() * scale_factor);
                world_linear_velocity.setZ(world_linear_velocity.getZ() * scale_factor);
                world_angular_velocity.setX(world_angular_velocity.getX() * scale_factor);
                world_angular_velocity.setY(world_angular_velocity.getY() * scale_factor);
                world_angular_velocity.setZ(world_angular_velocity.getZ() * scale_factor);
            }
            // vectorPrint("new linear:",world_linear_velocity);
        }
        
        // 使用旋转矩阵将线速度从世界坐标系转换到right_base坐标系
        tf::Vector3 base_linear_velocity = world_linear_velocity * world_to_base;
        tf::Vector3 base_angular_velocity = world_angular_velocity * world_to_base;
        
        _realtime_pub->msg_.linear.x = base_linear_velocity.getX();
        _realtime_pub->msg_.linear.y = base_linear_velocity.getY();
        _realtime_pub->msg_.linear.z = base_linear_velocity.getZ();
        _realtime_pub->msg_.angular.x = base_angular_velocity.getX();
        _realtime_pub->msg_.angular.y = base_angular_velocity.getY();
        _realtime_pub->msg_.angular.z = base_angular_velocity.getZ();
        // -----------------------------------------------------------
        if(press_flag){
            // matrixPrint("base:",world_to_base);
            // vectorPrint("world:",world_linear_velocity);
            // 使用 high_resolution_clock 获取当前时间点
            // auto now = std::chrono::high_resolution_clock::now();
        
            // // 将时间点转换为毫秒
            // auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        
            // // 计算从纪元（epoch）到现在的毫秒数
            // auto epoch = now_ms.time_since_epoch();
            // auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
        
            // 输出毫秒数
            // std::cout << "当前时间的毫秒数: " << value.count() << " 毫秒" << std::endl;
            // vectorPrint("linear:",base_linear_velocity);
            // vectorPrint("world:",world_angular_velocity);
            // vectorPrint("angular:",base_angular_velocity);
        }

        if(pgi_flag){
            RGI_handle(PGI_client,pgi_srv);
            pgi_srv.request.position=min(pgi_srv.request.position,999);
        }
        if(RGI_flag){
            RGI_handle(RGI_client,rgi_srv);
            rgi_srv.request.rel_angle=0;
        }

        // if(std::abs(last_base_linear_velocity.x()-base_linear_velocity.x())<1e-6&&
        //    std::abs(last_base_linear_velocity.y()-base_linear_velocity.y())<1e-6&&
        //    std::abs(last_base_linear_velocity.z()-base_linear_velocity.z())<1e-6&&
        //    std::abs(last_base_angular_velocity.x()-base_angular_velocity.x())<1e-6&&
        //    std::abs(last_base_angular_velocity.y()-base_angular_velocity.y())<1e-6&&
        //    std::abs(last_base_angular_velocity.z()-base_angular_velocity.z())<1e-6){
        //     // _realtime_pub->unlockAndPublish();//20ms时间间隔发送
        // }
        // else{
        //     std::cout<<"diff"<<std::endl;
        //     vectorPrint("last",last_base_linear_velocity);
        //     vectorPrint("now ",base_linear_velocity);
        //     _realtime_pub->unlockAndPublish();//20ms时间间隔发送
        // }
        // last_base_linear_velocity=base_linear_velocity;
        // last_base_angular_velocity=base_angular_velocity;

        _realtime_pub->unlockAndPublish();//20ms时间间隔发送
        if(target_control_mode==0)target_control_mode=-1;
    }
}

void ur5_js::Callback1(const sensor_msgs::Joy::ConstPtr joy)
{
    _joystick_ptr->x = joy->buttons[0];             //x
    _joystick_ptr->a = joy->buttons[1];             //a
    _joystick_ptr->b = joy->buttons[2];             //b
    _joystick_ptr->y = joy->buttons[3];             //y
    _joystick_ptr->LB = joy->buttons[4];            //LB
    _joystick_ptr->RB = joy->buttons[5];            //RB
    _joystick_ptr->LT = joy->buttons[6];            //LT
    _joystick_ptr->RT = joy->buttons[7];            //RT
    _joystick_ptr->BACK = joy->buttons[8];          //BACK
    _joystick_ptr->unknown1 = joy->buttons[9];      //START
    _joystick_ptr->unknown2 = joy->buttons[10];     //LS press
    _joystick_ptr->unknown3 = joy->buttons[11];     //RS press
    _joystick_ptr->button_l = joy->axes[0];         //left & right
    _joystick_ptr->button_up = joy->axes[1];        //up
    _joystick_ptr->rstick_up = joy->axes[2];        //RS
    _joystick_ptr->rstick_l = joy->axes[3];         //RS
    _joystick_ptr->lstick_up = joy->axes[4];        //LS
    _joystick_ptr->lstick_l = joy->axes[5];         //LS
    if (_joystick_ptr->BACK &&!back_pressed) {
        back_pressed = true;
        back_press_start_time = ros::Time::now();
    } else if (!_joystick_ptr->BACK) {
        back_pressed = false;
    }

    if (_joystick_ptr->unknown1 &&!start_pressed) {
        start_pressed = true;
        start_press_start_time = ros::Time::now();
    } else if (!_joystick_ptr->unknown1) {
        start_pressed = false;
    }
}

void ur5_js::twistMoveCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    twist_world_linear_velocity.setX(msg->data[0]);
    twist_world_linear_velocity.setY(msg->data[1]);
    twist_world_linear_velocity.setZ(msg->data[2]);
    twist_world_angular_velocity.setX(msg->data[3]);
    twist_world_angular_velocity.setY(msg->data[4]);
    twist_world_angular_velocity.setZ(msg->data[5]);
    bool flag=false;
    printf("get twist param:(%.4f,%.4f,%.4f)(%.4f,%.4f,%.4f)\n",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
    switch_mode=7;
    for(int i=0;i<6;i++)if(std::abs(msg->data[i])>1e-6){
        target_control_mode=1;
        flag=true;
        break;
    }
    if(!flag){
        target_control_mode=0;
    }
}

void freeControl()
{
    controller_manager_msgs::SwitchController switchRequest;
    ros::ServiceClient switchClient;
    ros::NodeHandle nh;

    if(Dual_arm_flag){
        std::string switch_goalrobot="left_robot";
        std::string switch_controller_name;
        switch_controller_name = switch_goalrobot + "/controller_manager/switch_controller";
        switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
        switchRequest.request.start_controllers.push_back("scaled_pos_joint_traj_controller");
        switchRequest.request.stop_controllers.push_back("twist_controller");
        switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        if (switchClient.call(switchRequest))
        {
            if (switchRequest.response.ok)
                ROS_INFO("free left controller was successful.");
            else
                ROS_INFO("free left controller failed");
        }
        else
            ROS_ERROR("Failed to call left controller switch service.");

        switch_goalrobot="right_robot";
        switch_controller_name = switch_goalrobot + "/controller_manager/switch_controller";
        switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
        switchRequest.request.start_controllers.push_back("scaled_pos_joint_traj_controller");
        switchRequest.request.stop_controllers.push_back("twist_controller");
        switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        if (switchClient.call(switchRequest))
        {
            if (switchRequest.response.ok)
                ROS_INFO("free left controller was successful.");
            else
                ROS_INFO("free left controller failed");
        }
        else
            ROS_ERROR("Failed to call left controller switch service.");
    }else{
        std::string switch_controller_name = "/controller_manager/switch_controller";
        switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
        switchRequest.request.start_controllers.push_back("scaled_pos_joint_traj_controller");
        switchRequest.request.stop_controllers.push_back("twist_controller");
        switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        if (switchClient.call(switchRequest))
        {
            if (switchRequest.response.ok)
                ROS_INFO("free left controller was successful.");
            else
                ROS_INFO("free left controller failed");
        }
        else
            ROS_ERROR("Failed to call left controller switch service.");

    }
}
void takeControl()
{
    controller_manager_msgs::SwitchController switchRequest;
    ros::ServiceClient switchClient;
    ros::NodeHandle nh;

    if(Dual_arm_flag){
        std::string switch_goalrobot="left_robot";
        std::string switch_controller_name;
        switch_controller_name = switch_goalrobot + "/controller_manager/switch_controller";
        switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
        switchRequest.request.start_controllers.push_back("twist_controller");
        switchRequest.request.stop_controllers.push_back("scaled_pos_joint_traj_controller");
        switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        if (switchClient.call(switchRequest))
        {
            if (switchRequest.response.ok)
                ROS_INFO("take left controller was successful.");
            else
                ROS_INFO("take left controller failed");
        }
        else
            ROS_ERROR("Failed to call left controller switch service.");

        switch_goalrobot="right_robot";
        switch_controller_name = switch_goalrobot + "/controller_manager/switch_controller";
        switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
        switchRequest.request.start_controllers.push_back("twist_controller");
        switchRequest.request.stop_controllers.push_back("scaled_pos_joint_traj_controller");
        switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        if (switchClient.call(switchRequest))
        {
            if (switchRequest.response.ok)
                ROS_INFO("take left controller was successful.");
            else
                ROS_INFO("take left controller failed");
        }
        else
            ROS_ERROR("Failed to call left controller switch service.");
    }else{
        std::string switch_controller_name = "/controller_manager/switch_controller";
        switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
        switchRequest.request.start_controllers.push_back("twist_controller");
        switchRequest.request.stop_controllers.push_back("scaled_pos_joint_traj_controller");
        switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        if (switchClient.call(switchRequest))
        {
            if (switchRequest.response.ok)
                ROS_INFO("take left controller was successful.");
            else
                ROS_INFO("take left controller failed");
        }
        else
            ROS_ERROR("Failed to call left controller switch service.");
    }
}
