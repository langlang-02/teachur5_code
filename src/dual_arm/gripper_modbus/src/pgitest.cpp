#include <ros/ros.h>
#include <gripper_modbus/Gripper.h>

bool RGI_handle(ros::ServiceClient client,gripper_modbus::Gripper srv){
    // 调用服务
    if (client.call(srv)) {
        if (srv.response.flag) {
            ROS_INFO("夹爪控制成功！");
            return 1;
        } else {
            ROS_ERROR("夹爪控制失败！");
        }
    } else {
        ROS_ERROR("服务调用失败！");
    }
    return 0;
}
int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "PGI_modbus_test");
    // 创建节点句柄
    ros::NodeHandle nh;

    ros::service::waitForService("/LPGI_Gripper_ModbusControl");
    ros::ServiceClient PGI_client = nh.serviceClient<gripper_modbus::Gripper>("/LPGI_Gripper_ModbusControl");
    ros::service::waitForService("/RGI_Gripper_ModbusControl");
    ros::ServiceClient right_pgi_client = nh.serviceClient<gripper_modbus::Gripper>("/RGI_Gripper_ModbusControl");

    // 创建服务请求对象
    gripper_modbus::Gripper srv;
    // 设置请求参数
    srv.request.parallel_force      = 100 ;//平行力值，20～100
    srv.request.parallel_velocity   = 50 ;//平行速度，1～100
    srv.request.rotate_force        = 20 ;//旋转力值，20～100
    srv.request.rotate_velocity     = 50 ;//旋转速度，1～100
    srv.request.abs_angle           = 99999999 ;//绝对角度，-160*32768-32768～160*32678+32767
    srv.request.rel_angle           = 0 ;//相对角度，-32768～32767
    srv.request.position            = 0 ;//平行位置，0～1000
    srv.request.block_flag          = true ;//阻塞标志
    srv.request.stop_flag           = false ;//强制停止
    // RGI_handle(PGI_client,srv);
    // RGI_handle(right_pgi_client,srv);

    // srv.request.stop_flag=1;
    // srv.request.rel_angle=3600;
    // RGI_handle(right_pgi_client,srv);
    // return 0;
    while(1){
        printf("left pgi position:");
        scanf("%d",&srv.request.position);
        RGI_handle(PGI_client,srv);

        // printf("right RGI position:");
        // scanf("%d",&srv.request.position);
        // printf("right RGI rel_angle:");
        // scanf("%d",&srv.request.rel_angle);
        // printf("right RGI abs_angle:");
        // scanf("%d",&srv.request.abs_angle);
        // printf("right RGI rotate_force:");
        // scanf("%d",&srv.request.rotate_force);
        // printf("right RGI rotate_velocity:");
        // scanf("%d",&srv.request.rotate_velocity);
        // RGI_handle(right_pgi_client,srv);
    }

    return 0;
}
