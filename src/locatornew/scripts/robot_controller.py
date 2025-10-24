#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
import json
from std_msgs.msg import String
import numpy as np
import tf
from icecream import ic
from typing import Optional, Tuple
import time

class Robot:
    def __init__(self):
        # 状态标志
        self.moveP_done = False  # 运动完成标志
        self.getpose_done = False  # 位姿获取完成标志
        self.cmd_pose = None  # 存储指令获取的位姿

        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # ROS话题
        self.pub = rospy.Publisher('asynOperation_in', String, queue_size=10)
        self.sub = rospy.Subscriber('asynOperation_out', String, self.feedback_callback)
        time.sleep(1)
        # 配置参数
        self.operation_name = "locator_operation"
        self.station_name = "locator_station"
        self.arm_tool_map = {"L": "left_tool0", "R": "right_tool0"}  # 机械臂与工具坐标系映射

    def getPoseBase(self, arm: str = "L") -> Optional[np.ndarray]:
        """
        获取机械臂位姿（双方式）并对比
        :param arm: 机械臂标识，默认"L"
        :return: 指令获取的位姿和TF获取的位姿
        """
        # 1. 通过"P L"或"P R"指令获取位姿
        cmd_pose = self._get_pose_by_command(arm)
        return cmd_pose
        
        # 2. 通过TF获取位姿
        tf_pose = self._get_pose_by_tf(arm)
        
        # 3. 对比两种方式获取的位姿
        if cmd_pose is not None and tf_pose is not None:
            self._compare_poses(cmd_pose, tf_pose, arm)
            
        return cmd_pose, tf_pose

    def _get_pose_by_command(self, arm: str) -> Optional[np.ndarray]:
        """通过指令获取位姿"""
        # 构建指令
        command = f"P {arm}"
        msg_data = {
            "curr_station": self.station_name,
            "operation": self.operation_name,
            "order": command
        }
        msg = String(json.dumps(msg_data))
        
        # 发送指令并等待结果
        self.getpose_done = False
        self.cmd_pose = None
        self.pub.publish(msg)
        # print(f"已发送位姿获取指令: {msg_data}")
        
        # 等待反馈（超时5秒）
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        rate.sleep()
        while not self.getpose_done and not rospy.is_shutdown() and (time.time()-start_time)<5.0:
            rate.sleep()
            print(f"正在获取{arm}位姿....")
        if not self.getpose_done:
            rospy.logerr(f"获取{arm}位姿超时")
            exit(1)
            
        return self.cmd_pose

    def _get_pose_by_tf(self, arm: str) -> Optional[np.ndarray]:
        """通过TF变换获取位姿"""
        target_frame = self.arm_tool_map.get(arm, "left_tool0")
        source_frame = "world"
        
        try:
            # 等待TF变换
            self.tf_listener.waitForTransform(
                source_frame, target_frame, rospy.Time(0), rospy.Duration(4.0)
            )
            # 获取变换
            (trans, rot) = self.tf_listener.lookupTransform(
                source_frame, target_frame, rospy.Time(0)
            )
            
            # 组合为[x, y, z, qx, qy, qz, qw]格式
            return np.array([
                trans[0], trans[1], trans[2],
                rot[0], rot[1], rot[2], rot[3]
            ])
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF获取{arm}位姿失败: {str(e)}")
            return None

    def _compare_poses(self, cmd_pose: np.ndarray, tf_pose: np.ndarray, arm: str):
        """对比两种方式获取的位姿"""
        # 计算位置差异（米 -> 毫米）
        pos_diff = np.abs(cmd_pose[:3] - tf_pose[:3]) * 1000
        # 计算姿态差异（四元数）
        quat_diff = np.abs(cmd_pose[3:] - tf_pose[3:])
        
        rospy.logwarn(f"\n===== {arm}机械臂位姿对比 =====")
        rospy.logwarn(f"指令获取: {np.round(cmd_pose, 6)}")
        rospy.logwarn(f"TF获取  : {np.round(tf_pose, 6)}")
        rospy.logwarn(f"位置差异(mm): {np.round(pos_diff, 3)}")
        rospy.logwarn(f"姿态差异    : {np.round(quat_diff, 6)}")
        rospy.logwarn("===========================\n")

    def moveP(self, speed: float, acce: float, pose: np.ndarray, arm: str = "L"):
        """控制机械臂运动到指定位姿"""
        rate = rospy.Rate(3)
        
        # 处理姿态：弧度转角度
        pose_list = list(pose[:3]) + list(pose[3:])
        # pose_list[0]+=0.03
        
        # 构建运动指令
        script = f"a {arm} PTP P {speed} {acce} {' '.join(map(str, np.round(pose_list, 6)))}"
        msg_data = {
            "curr_station": self.station_name,
            "operation": self.operation_name,
            "order": script
        }
        operation_in = String(json.dumps(msg_data, sort_keys=True, indent=4))
        
        # 发送指令并等待完成
        print(f"发送运动指令: {script}")
        # return 
        self.moveP_done = False
        self.pub.publish(operation_in)
        
        while not self.moveP_done and not rospy.is_shutdown():
            rate.sleep()
        time.sleep(2.0)

    def feedback_callback(self, msg: String):
        """处理反馈消息"""
        # print("\n===== 收到反馈 =====")
        # print(msg.data)
        # print("===================\n")
        
        try:
            # 解析JSON格式反馈
            feedback = json.loads(msg.data)
            # 处理位姿反馈
            if feedback.get("operation") == "locator_operation" and "result" in feedback:
                if feedback.get("order")[0] == "P":
                    # 解析7位姿结果
                    pose_str = feedback["result"]
                    pose = np.array([float(x) for x in pose_str.split(',')])
                    if len(pose) == 7:
                        self.cmd_pose = pose
                        self.getpose_done = True
                        # print("成功解析位姿反馈")
                elif feedback.get("order")[0] == "a":
                    result=feedback["result"]
                    if result=="success":
                        print(f"运动到目标点成功")
                    else:
                        rospy.logerr(f"运动到目标点失败 result={result}")
                    self.moveP_done = True
                
        except json.JSONDecodeError:
            rospy.logwarn("反馈不是JSON格式")
        except Exception as e:
            rospy.logerr(f"解析反馈出错: {str(e)}")

    def close(self):
        pass

if __name__ == '__main__':
    rospy.init_node('robot_controller_node', anonymous=True)
    robotctl = Robot()
    rospy.sleep(1)  # 等待初始化完成
    
    # 循环获取左右臂位姿并对比
    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        # 获取左臂位姿
        print("----- 获取左臂位姿 -----")
        cmd_pose_L = robotctl._get_pose_by_command(arm="L")
        ic("左臂位姿:", cmd_pose_L)
        
        # 获取右臂位姿
        # print("----- 获取右臂位姿 -----")
        # cmd_pose_R, tf_pose_R = robotctl.getPoseBase(arm="R")
        # ic("右臂指令位姿:", cmd_pose_R)
        # ic("右臂TF位姿:", tf_pose_R)
        
        rate.sleep()
    
    robotctl.close()
