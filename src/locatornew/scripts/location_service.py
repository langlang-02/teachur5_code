#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
import numpy as np
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import cv2
import pyrealsense2 as rs2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32MultiArray
# 导入自定义服务类型
from locatornew.srv import Location, LocationRequest, LocationResponse  # 包含新增的Locator
from capture import *
from analyze import *
import shutil
import traceback
import re  # 新增：处理xacro文件时需要
from icecream import ic
# 设备名称与配置文件的映射关系
deviceNameToConfigFile = {
    'uv': 'config/紫外光谱仪.json',
    'fume_hood': 'config/通风橱.json',
    'auto_balance': 'config/自动进样天平.json',
}


class Locator:
    """定位器类，同时支持Topic和Service两种请求方式"""
    
    def __init__(self):
        """初始化节点，创建Topic和Service相关组件"""
        print("尝试初始化节点：location_service（支持Topic和Service）")
        self.config = None
        self.pose = None
        self.moveAndCapture = None
        self.pattern = None

        # 结果缓存变量
        self.cached_bTt = None  # 4x4变换矩阵
        self.cached_station = None
        self.publish_timer = None  # 10Hz发布定时器

        # 初始化ROS节点
        rospy.init_node('location_service', anonymous=False)

        # Topic相关
        self.obj_bTt_pub = rospy.Publisher('/obj_to_robot_holdon', PoseStamped, queue_size=10)
        self.obj_name_pub = rospy.Publisher('/class_order_holdon', String, queue_size=10)
        self.locator_sub = rospy.Subscriber('/locator_topic', String, self.locator_topic_callback)

        # Service相关
        self.locator_service = rospy.Service(
            '/locator_service',
            Location,
            self.locator_service_callback
        )

        print("节点初始化完成：location_service（支持Topic和Service）")
        print("订阅话题：/locator_topic")
        print("提供服务：/locator_service")

    def shutdown(self):
        """关闭节点资源"""
        print("\n关闭节点...")
        if self.publish_timer is not None and self.publish_timer.is_alive():
            self.publish_timer.shutdown()
            rospy.loginfo("10Hz发布定时器已停止")

    def oneloc(self):
        """单次识别逻辑"""
        print('~~~~~~~~开始单次识别~~~~~~~~')
        try:
            self.moveAndCapture.savePhotoAndPose(0, True)
            result_bTt = self.pattern.realtime(
                self.moveAndCapture.camera.saveFrameTo('./img_take/oneloc.png'),
                self.moveAndCapture.robot.getPoseBase()
            )
            ic(result_bTt)
            
            # 处理Aruco字典格式
            if isinstance(result_bTt, dict) and len(result_bTt) > 0:
                result_bTt = next(iter(result_bTt.values()))
            print(f"~~~~~~~~结束单次识别~~~~~~~~")
            return result_bTt
        except Exception as e:
            rospy.logerr(f"单次识别失败：{traceback.format_exc()}")
            return None

    def realtime(self):
        """实时识别逻辑"""
        print('~~~~~~~~开始实时识别~~~~~~~~')
        try:
            while not rospy.is_shutdown():
                result_bTt = self.pattern.realtime(
                    self.moveAndCapture.camera.saveFrameTo('./img_take/oneloc.png'),
                    self.moveAndCapture.robot.getPoseBase()
                )
            print(f"~~~~~~~~结束实时识别~~~~~~~~")
        except Exception as e:
            rospy.logerr(f"实时识别失败：{traceback.format_exc()}")

    def publish_holdon_data(self, event):
        """10Hz定时器回调：发布缓存的位姿和站点信息"""
        if self.cached_bTt is None or self.cached_station is None:
            rospy.logwarn_throttle(1, "无有效缓存的bTt或station，跳过发布")
            return
        
        # 转换bTt为PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = rospy.Time.now()
        
        # 位置信息
        pose_msg.pose.position.x = self.cached_bTt[0, 3]
        pose_msg.pose.position.y = self.cached_bTt[1, 3]
        pose_msg.pose.position.z = self.cached_bTt[2, 3]
        
        # 姿态信息（旋转矩阵转四元数）
        rot_matrix = self.cached_bTt[:3, :3]
        quat = R.from_matrix(rot_matrix).as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # 发布消息
        self.obj_bTt_pub.publish(pose_msg)
        self.obj_name_pub.publish(self.cached_station)

    def merge_configs(self, base_config, override_config):
        """递归合并配置字典"""
        merged = base_config.copy()
        for key, value in override_config.items():
            if isinstance(value, dict) and key in merged and isinstance(merged[key], dict):
                merged[key] = self.merge_configs(merged[key], value)
            else:
                merged[key] = value
        return merged

    def _common_location_logic(self, command_dict):
        """通用定位逻辑（Topic和Service共用）"""
        try:
            # 检查station参数
            if 'station' not in command_dict:
                status = "error: the command requires 'station' value"
                rospy.logerr(status)
                return (status, False, None)
            station = command_dict['station']
            self.cached_station = station

            # 检查设备是否在配置中
            if station not in deviceNameToConfigFile:
                available_stations = list(deviceNameToConfigFile.keys())
                status = f"error: unknown station '{station}', available stations: {available_stations}"
                rospy.logerr(status)
                return (status, False, None)
            deviceConfigFile = deviceNameToConfigFile[station]

            # 读取基础配置文件
            try:
                with open(deviceConfigFile, 'r', encoding='utf-8') as f:
                    base_config = json.load(f)
                rospy.loginfo(f"loaded basic config from {deviceConfigFile}")
            except Exception as e:
                status = f"error: failed to load basic config - {str(e)}"
                rospy.logerr(status)
                return (status, False, None)

            # 合并配置
            self.config = self.merge_configs(base_config, command_dict)
            print(json.dumps(self.config, indent=4, ensure_ascii=False))

            # 初始化移动捕获和标定板
            self.moveAndCapture = MoveAndCapture(self.config)
            marker_type = self.config.get('type', '')
            task = self.config.get('task', '')

            # 初始化标定板类型
            if marker_type == 'aruco':
                self.pattern = Aruco(self.config)
            elif marker_type == 'charuco':
                self.pattern = Charuco(self.config)
            elif marker_type == 'circleGrid':
                self.pattern = CircleGrid(self.config)
            else:
                status = f"error: unknown board type '{marker_type}'"
                rospy.logerr(status)
                return (status, False, None)

            self.moveAndCapture.publish_tf = self.pattern.publish_tf
            self.moveAndCapture.gTc = self.pattern.gTc
            print(f'~~~~~config info ({deviceConfigFile})~~~~~')

            # 处理不同任务
            task_result_bTt = None
            if task == 'oneloc':
                task_result_bTt = self.oneloc()
                if task_result_bTt is None:
                    status = "error: single-location failed (no valid bTt)"
                    rospy.logerr(status)
                    return (status, False, None)
                status = "success: single-location completed"

            elif task == 'realtime':
                self.realtime()
                status = "success: realtime location started (no cached bTt)"
                return (status, False, None)

            elif task in ['multiloc', 'calib']:
                # 执行单次识别获取初始bTt
                oneloc_bTt = self.oneloc()
                if oneloc_bTt is None:
                    status = "error: multi-location/calibration failed (initial single-location failed)"
                    rospy.logerr(status)
                    return (status, False, None)
                if isinstance(oneloc_bTt, dict) and len(oneloc_bTt) > 0:
                    oneloc_bTt = next(iter(oneloc_bTt.values()))

                # 移动捕获
                self.moveAndCapture.moveAndCaptureAll(oneloc_bTt)

                # 处理多次定位
                if task == 'multiloc':
                    task_result_bTt = self.pattern.multiloc()
                    if task_result_bTt is None:
                        status = "error: multi-location failed (no valid average bTt)"
                        rospy.logerr(status)
                        return (status, False, None)
                    status = "success: multi-location completed"

                # 处理标定
                elif task == 'calib':
                    gTc = self.pattern.calib()
                    if gTc is None:
                        status = "error: calibration failed"
                        rospy.logerr(status)
                        return (status, False, None)
                    if self.config.get('calib', {}).get('overwrite', False):
                        self.write_gTc_to_xacro(gTc)
                    status = "success: calibration completed (no cached bTt)"
                    return (status, False, None)

            else:
                status = f"error: unknown task '{task}' (supported: oneloc/multiloc/calib/realtime)"
                rospy.logerr(status)
                return (status, False, None)

            # 缓存结果并启动发布定时器
            self.cached_bTt = task_result_bTt
            if self.publish_timer is None or not self.publish_timer.is_alive():
                self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.publish_holdon_data)
                rospy.loginfo("10Hz publishing timer started (publishing obj_to_robot_holdon and class_order_holdon)")

            return (status, True, task_result_bTt)

        except Exception as e:
            status = f"error: failed to execute common logic - {str(e)}\n{traceback.format_exc()}"
            rospy.logerr(status)
            return (status, False, None)

    def locator_topic_callback(self, _data):
        """Topic回调：处理/locator_topic话题请求"""
        rospy.loginfo("\n收到Topic请求：/locator_topic")
        try:
            command_dict = json.loads(_data.data)
            rospy.loginfo(f"Topic命令内容：{command_dict}")
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析Topic命令失败：{str(e)}，原始数据：{_data.data}")
            return

        # 调用通用逻辑
        status, _, _ = self._common_location_logic(command_dict)
        rospy.loginfo(f"Topic请求处理结果：{status}")

    def locator_service_callback(self, req):
        """Service回调：处理/locator_service服务请求"""
        rospy.loginfo("\n收到Service请求：/locator_service")
        resp = LocationResponse()  # 初始化响应对象

        try:
            # 解析服务请求参数
            command_dict = json.loads(req.command_json)
            rospy.loginfo(f"Service命令内容：{command_dict}")
        except json.JSONDecodeError as e:
            resp.status = f"illegal json - {str(e)}"
            resp.result_valid = False
            resp.cached_bTt = []
            resp.cached_station = ""
            return resp

        # 调用通用逻辑
        status, result_valid, task_result_bTt = self._common_location_logic(command_dict)

        # 构造服务响应
        resp.status = status
        resp.result_valid = result_valid
        resp.cached_station = self.cached_station if self.cached_station else ""

        # 处理bTt矩阵（展平为16元素列表）
        if result_valid and task_result_bTt is not None:
            try:
                # 确保bTt是4x4矩阵
                if isinstance(task_result_bTt, np.ndarray) and task_result_bTt.shape == (4, 4):
                    resp.cached_bTt = task_result_bTt.flatten().tolist()
                else:
                    resp.cached_bTt = []
                    resp.status += "warning,the bTt is not 4x4 matrix"
            except Exception as e:
                resp.cached_bTt = []
                resp.status += f"bTt trans failed：{str(e)}"

        return resp

    def write_gTc_to_xacro(self, gTc):
        """写入标定结果到Xacro文件"""
        try:
            arm_robot_description = os.popen("rospack find arm_robot_description").read().strip()
            xacro_path = os.path.join(arm_robot_description, "urdf", "dual_arm_robot.xacro")
            if not os.path.exists(xacro_path):
                rospy.logerr(f"Xacro文件不存在: {xacro_path}")
                return False
            backup_path = f"{xacro_path}.bak"
            shutil.copy2(xacro_path, backup_path)
            rospy.loginfo(f"已备份原文件至: {backup_path}")
        except Exception as e:
            rospy.logerr(f"文件准备失败: {e}")
            return False

        # 解析gTc矩阵
        xyz = [round(v, 6)/1000 for v in gTc[:3, 3].tolist()]
        rot_matrix = gTc[:3, :3]
        r = R.from_matrix(rot_matrix)
        rpy = [round(v, 6) for v in r.as_euler('xyz', degrees=False)]

        # 目标关节名称
        if self.config.get('arm', 'left') == 'left':
            target_joint_name = "${prefix2}tool0_to_camera_color_optical_frame"
        else:
            target_joint_name = "${prefix1}tool0_to_camera_color_optical_frame"
        target_joint_found = False

        # 处理文件内容
        new_content = []
        try:
            with open(backup_path, 'r') as f:
                lines = f.readlines()

            in_target_joint = False
            in_origin_block = False
            
            for line in lines:
                if f'joint name="{target_joint_name}"' in line and 'type="fixed"' in line:
                    target_joint_found = True
                    in_target_joint = True
                    new_content.append(line)
                    continue

                if in_target_joint:
                    if "<origin" in line and "/>" not in line:
                        in_origin_block = True
                        new_content.append(line)
                        continue
                    
                    if in_origin_block:
                        if "xyz=" in line:
                            new_line = re.sub(
                                r'xyz="[^"]*"',
                                f'xyz="{xyz[0]} {xyz[1]} {xyz[2]}"',
                                line
                            )
                            new_content.append(new_line)
                        elif "rpy=" in line:
                            new_line = re.sub(
                                r'rpy="[^"]*"',
                                f'rpy="{rpy[0]} {rpy[1]} {rpy[2]}"',
                                line
                            )
                            new_content.append(new_line)
                        elif "/>" in line or "</origin>" in line:
                            new_content.append(line)
                            in_origin_block = False
                        else:
                            new_content.append(line)
                        continue

                    new_content.append(line)
                    
                    if "</joint>" in line:
                        in_target_joint = False
                else:
                    new_content.append(line)

            # 未找到关节时添加新关节
            if not target_joint_found:
                rospy.loginfo(f"未找到关节{target_joint_name}，添加新关节")
                new_joint = [
                    f'  <joint name="{target_joint_name}" type="fixed">\n',
                    f'    <origin\n',
                    f'      xyz="{xyz[0]} {xyz[1]} {xyz[2]}"\n',
                    f'      rpy="{rpy[0]} {rpy[1]} {rpy[2]}" />\n',
                    f'    <parent\n',
                    f'      link="${{prefix}}tool0" />\n',
                    f'    <child\n',
                    f'      link="${{prefix}}camera_color_optical_frame" />\n',
                    f'    <axis\n',
                    f'      xyz="0 0 0" />\n',
                    f'  </joint>\n'
                ]
                # 插入到xacro:macro结束前
                for i in range(len(new_content)):
                    if "</xacro:macro>" in new_content[i]:
                        new_content = new_content[:i] + new_joint + new_content[i:]
                        break

            # 写入文件
            if self.config.get('calib', {}).get('overwrite', False):
                with open(xacro_path, 'w') as f:
                    f.writelines(new_content)
                rospy.loginfo(f"已写入新Xacro文件: {xacro_path}")
                return True
            else:
                rospy.loginfo("overwrite为false，不写入文件")
                return True

        except Exception as e:
            rospy.logerr(f"写入失败: {traceback.format_exc()}")
            shutil.copy2(backup_path, xacro_path)
            return False


if __name__ == '__main__':
    """主函数：启动节点并保持运行"""
    try:
        locator = Locator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        locator.shutdown()
    