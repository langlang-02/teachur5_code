#include <ros/ros.h>
#include "balance_com/Balance.h"
#include <iostream>
#include <cstdlib>

// 发送去皮请求
bool tareRequest(ros::ServiceClient& client) {
    balance_com::Balance srv;
    srv.request.operation = "tare";
    if (client.call(srv)) {
        return srv.response.success;
    } else {
        ROS_ERROR("Failed to call service for tare operation");
        return false;
    }
}

// 发送读取重量请求
double readWeightRequest(ros::ServiceClient& client) {
    balance_com::Balance srv;
    srv.request.operation = "weight";
    if (client.call(srv)) {
        return srv.response.weight;
    } else {
        ROS_ERROR("Failed to call service for read weight operation");
        return -1.0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "balance_test");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<balance_com::Balance>("/Balance_com");
    std::string choice;
    std::cout << "是否需要去皮？(y/n): ";
    std::cin >> choice;

    if (choice == "y" || choice == "Y") {
        for(int i=0;i<3;i++)
            if (tareRequest(client)) {
                std::cout << "去皮操作成功" << std::endl;
            } else {
                std::cout << "去皮操作失败" << std::endl;
            }
    }

    try {
        while (ros::ok()) {
            double weight = readWeightRequest(client);
            printf("当前天平称量质量: %.1f 毫克\n",weight);
            ros::Duration(0.1).sleep();
        }
    } catch (...) {
        std::cout << "程序终止" << std::endl;
    }

    return 0;
}