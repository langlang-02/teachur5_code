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

    std::string Robot_operation::grab_test(const std::vector<std::any>& args){
        char id = std::any_cast<double>(args[0])+'0';

        std::string result;
        result=Analysis_Command("G 0 1000 0 0",false);
        if(result!="idle idle")return result;
        result=Analysis_Command("A S PTP J 0.2 0.2 reset",false);
        if(result!="success")return result;
        printf("id=%d\n",id);
        result=Analysis_Command(replaceChar("A S PTP J 0.2 0.2 block@_high",'@',id),false);
        if(result!="success")return result;
        result=Analysis_Command(replaceChar("A S PTP J 0.2 0.2 block@_low",'@',id),false);
        if(result!="success")return result;
        result=Analysis_Command("G -1 0 0 99999999",false);
        if(result!="idle clamping")return result;
        result=Analysis_Command(replaceChar("A S LIN J 0.05 0.05 block@_high",'@',id),false);
        if(result!="success")return result;
        result=Analysis_Command(replaceChar("A S PTP J 0.2 0.2 block4_high",'@',id),false);
        if(result!="success")return result;
        result=Analysis_Command("G -1 -1 360 99999999",false);
        if(result!="idle clamping")return result;
        result=Analysis_Command("A S PTP J 0.2 0.2 block_pgi_high",false);
        if(result!="success")return result;
        result=Analysis_Command("A S PTP J 0.2 0.2 block_pgi_mid",false);
        if(result!="success")return result;
        result=Analysis_Command("A S LIN J 0.05 0.05 block_pgi_low",false);
        if(result!="success")return result;
        result=Analysis_Command("G 1000 -1 0 99999999",false);
        if(result!="clamping clamping")return result;
        result=Analysis_Command("G -1 1000 0 99999999",false);
        if(result!="clamping idle")return result;
        result=Analysis_Command("A S LIN J 0.1 0.1 block_pgi_high",false);
        if(result!="success")return result;
        result=Analysis_Command("A S PTP J 0.2 0.2 reset",false);
        if(result!="success")return result;

        return "success";
    }



    std::string Robot_operation::take_test(const std::vector<std::any>& args){
        char id = std::any_cast<double>(args[0])+'0';

        std::string result;
        result=Analysis_Command("A S PTP J 0.2 0.2 reset",false);
        if(result!="success")return result;
        printf("id=%d\n",id);
        result=Analysis_Command("A S LIN J 0.1 0.1 block_pgi_high",false);
        if(result!="success")return result;
        result=Analysis_Command("A S LIN J 0.05 0.05 block_pgi_low",false);
        if(result!="success")return result;
        result=Analysis_Command("G -1 0 0 99999999",false);
        if(result!="clamping clamping")return result;
        result=Analysis_Command("G 0 -1 0 99999999",false);
        if(result!="idle clamping")return result;
        result=Analysis_Command("A S LIN J 0.1 0.1 block_pgi_high",false);
        if(result!="success")return result;
        result=Analysis_Command("A S PTP J 0.2 0.2 block4_high",false);
        if(result!="success")return result;
        result=Analysis_Command(replaceChar("A S PTP J 0.2 0.2 block@_high",'@',id),false);
        if(result!="success")return result;
        result=Analysis_Command(replaceChar("A S LIN J 0.05 0.05 block@_low",'@',id),false);
        if(result!="success")return result;
        result=Analysis_Command("G -1 1000 0 99999999",false);
        if(result!="idle idle")return result;
        result=Analysis_Command(replaceChar("A S PTP J 0.2 0.2 block@_high",'@',id),false);
        if(result!="success")return result;
        result=Analysis_Command("A S PTP J 0.2 0.2 reset",false);
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