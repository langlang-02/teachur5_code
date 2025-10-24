#include "manipulator/robot_arm.h"
namespace Robot_capsulation
{
    std::string replaceChar(const std::string& str, char oldChar, char newChar) {
        std::string result = str;
        for (char& c : result) {
            if (c == oldChar) {
                c = newChar;
            }
        }
        return result;
    }
    // 类似fakecom执行带参函数
    std::string Robot_operation::move_test(const std::vector<std::any>& args){
        char id = std::any_cast<double>(args[0])+'0';

        std::string result;
        result=Analysis_Command("A S PTP J 0.2 0.2 reset",false);
        if(result!="success")return result;
        printf("id=%d\n",id);
        result=Analysis_Command(replaceChar("A S PTP J 0.2 0.2 block@_high",'@',id),false);
        if(result!="success")return result;
        return "success";
    }
    // 获取关节较并执行关节运动
    std::string Robot_operation::move_joints_test(void)
    {
        std::vector<double> joint_inverse;
        if(!Record_tool::get_joint("reset_inverse", joints_, joint_inverse))
            return "error";
        if(!arm_robot_ptr->move_targetJoints(joint_inverse,true,true))
            return "error";
        return "success";
    }

}