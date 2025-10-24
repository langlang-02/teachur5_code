#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from ast import While,literal_eval
import rospy
import json
from std_msgs.msg import String
import os  # 添加 os 模块导入
import time
import re
# from ImageSave import ImageSave
# ImageSave.ImageSave()
##################################################################################
class Station_command:
    def __init__(self, station_name):
        # 工作站名称
        self.station_name = station_name
        # 每次运动的完成标志
        # self.done_flag = True
        # rospy.set_param("/command_finish_flag", self.done_flag)
        self.done_flag = False
        self.done_msg = ""
        self.result=None
        self.result_list=[]

        script_path = os.path.dirname(os.path.abspath(__file__))
        file_path = script_path + "/stationdata/" + station_name + ".json"
        self.existstation_flag, self.station_data = self.read_json(file_path)
        # 确认是否存在gelstation
        if self.existstation_flag == True:
            print(self.station_name + " exists in workstation")
            print("Wait to topic created......")
            self.pub = rospy.Publisher('stationOperation_in', String, queue_size=1)
            rospy.Subscriber('stationOperation_out', String, self.feedback_callback)
            rospy.sleep(rospy.Duration(1.0))
            print("The topic have created !")
        else:
            print(self.station_name +" does not exist in workstation")


    # 命令发布与回调
    def test_command_pub(self, operation_name: str, force: bool):
        '''
        operation_name后可以跟@....$.....#....形式的参数
        其中@和$会将参数替换到每一行指令的@和$位置上
        #会将参数接在每条指令尾部，通常作是含参函数的参数列表
        单指令command后可以跟%.....形式字符串，表示执行期望结果，通常为夹爪使用判断是否夹持堵转
        '''
        
        self.result_list=[]
        # 判断是否为单行动作指令（以大写字母+空格开头）
        is_single_command = bool(re.match(r'^[A-Za-z] ', operation_name))
        
        args_str = ""
        matches = re.findall(r'([@$#])([^@$#]+)', operation_name)
        replace_map = {key: value for key, value in matches}
        keys = list(replace_map.keys())
        
        if len(keys) > 0:
            print('replace_map:', replace_map)
        
        # 填充缺失的占位符为空白
        for i in "@$#":
            if i not in keys:
                replace_map[i] = ''
        
        # 处理带连续参数的指令
        for i in "@$#":
            if i in keys:
                parts = operation_name.split(i, 1)
                operation_name = parts[0] + i
                break
        
        if '#' in keys:
            replace_map['#'] = ' ' + replace_map['#']
        
        # 准备要执行的脚本列表
        scripts_to_execute = []
        target_operation = "single_command" if is_single_command else operation_name
        
        if is_single_command:
            # 单行动作：直接使用输入的指令作为脚本
            print(f"执行单行动作指令: {operation_name}")
            scripts_to_execute = [operation_name]
        else:
            # 动作组：从station_data中获取对应脚本列表
            if not self.check_name_exists(self.station_data, operation_name):
                print(f"动作组 {operation_name} 不在 {self.station_name} 中")
                return "error"
            
            print(f"在 {self.station_name} 中找到动作组: {operation_name}")
            for command in self.station_data['cmd']:
                if command['name'] == operation_name:
                    print(f"执行动作组: {operation_name}")
                    # 过滤掉包含'W -1'的脚本
                    scripts_to_execute = [s for s in command['scripts']]
                    # scripts_to_execute = [s for s in command['scripts'] if 'W -1' not in s]
                    break
        
        # 发布所有脚本指令
        rate = rospy.Rate(1)
        for script in scripts_to_execute:
            # 替换脚本中的占位符
            script_parts = script.split('%',1)
            script=script_parts[0].strip()
            if len(script_parts)>1:
                target_results=[item.strip() for item in script_parts[1].split('%')]
            else:
                target_results=[]
            processed_script = script.replace('@', replace_map['@']).replace('$', replace_map['$']) + replace_map['#']
            obsOperation_in = {
                "curr_station": self.station_name,
                "operation": target_operation,
                "order": processed_script
            }
            operation_in = String(json.dumps(
                obsOperation_in, sort_keys=True, indent=4, separators=(',', ': ')))
            
            # 发布命令
            print('=============执行指令===============\n')
            # print(operation_in.data)
            print(f"    operation: {obsOperation_in['operation']}")
            print(f"    order: {obsOperation_in['order']}")
            # print('===================================\n')
            
            # continue_or_not()
            self.done_flag = False
            self.pub.publish(operation_in)
            
            # 等待执行完成
            while not self.done_flag and not rospy.is_shutdown():
                rate.sleep()
            
            # 检查结果是否匹配
            if (self.result['curr_station'] == obsOperation_in['curr_station'] and
                self.result['operation'] == obsOperation_in['operation'] and
                self.result['order'] == obsOperation_in['order']):
                self.result=self.result['result']
                if "error" in self.result or "offline" in self.result:
                    rospy.logerr(f"terminate all operations when an error occured!!! ({self.result})")
                    if True or input('continue Y/N:') not in "Yy":
                        print(json.dumps({"code": 500, "spe_msg_sign": "failed"}))
                        exit(1)
                    rospy.loginfo("continue to execute operations")
                if len(target_results)>0:
                    print(f"    target: {target_results}")
                    if self.result not in target_results:
                        rospy.logerr(f"terminate all operations when an unexpected result occured!!!\n({self.result}) not in {target_results}")
                        if "Exp_trans_take_bottle" in obsOperation_in['operation']:
                            run("Exp_trans_reset")
                        if True or input('continue Y/N:') not in "Yy":
                            print(json.dumps({"code": 500, "spe_msg_sign": "failed"}))
                            exit(1)
                        rospy.loginfo("continue to execute operations")
                self.result_list.append(self.result)
            else:
                return "error"
            print('\n===================================\n')
        
        return "error"

    def feedback_callback(self, msg):
        # print('=============rev msg===============')
        # print(msg.data)
        # print('===================================\n')
        self.result=None
        try:
            # 尝试将字符串转换为字典
            self.result = literal_eval(msg.data)
            if not isinstance(self.result, dict):
                raise TypeError("转换结果不是字典类型")
            self.done_flag=True
            print(f"    result: {self.result['result']}")
            return self.result
        except SyntaxError:
            # 语法错误：字符串格式不符合Python语法
            rospy.logerr(f"字典字符串语法错误，无法转换: {msg.data}")
        except TypeError as e:
            # 类型错误：转换结果不是字典
            rospy.logerr(f"转换失败: {str(e)}，输入字符串: {msg.data}")
        except Exception as e:
            # 其他未知异常
            rospy.logerr(f"转换字典时发生未知错误: {str(e)}，输入字符串: {msg.data}")
        self.done_flag = True

    def stop_command(self):
        self.stop_flag = rospy.get_param("/operate_stop_flag", False)
        if self.stop_flag == True:
            print("The station is stopped......")
            while(self.stop_flag == True):
                self.stop_flag = rospy.get_param("/operate_stop_flag", False)

    # 读取 JSON 文件
    def read_json(self, file_path):
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                data = json.load(file)  # 解析 JSON 文件
                return True, data
        except FileNotFoundError:
            return False, None
        except json.JSONDecodeError:
            return False, None

    # 检查 name 为 "take" 的命令是否存在
    def check_name_exists(self, json_data, name):
        if 'cmd' in json_data:
            for command in json_data['cmd']:
                if command['name'] == name:
                    return True
        return False
##################################################################################
def continue_or_not():
    if input('continue Y/N:') not in "Yy":
        pass
def run(op,force=False):
    # continue_or_not()
    return ops.test_command_pub(op,force)
if __name__ == '__main__':
    rospy.init_node('fake_command', anonymous=True)
    Run_totalnum = 1
    Run_currentnum = 0
    is_succ = True
    try:
        # ops = Station_command("screwstation")
        # ops = Station_command("122spectrometer")
        # ops = Station_command("305exp")
        # ops = Station_command("auto_balance")
        # ops = Station_command("fume_hood")
        ops = Station_command("table")

        while(ops.existstation_flag):
            run('o move_test 4')
            run('O move_joints_test')
            # # 全部指令解析详见src/station_func.cpp
            # # 加载data/robot_JP.json点位数据
            # run("F J robot")
            # # 机械臂运动 A S (PTP/LIN/RRT) J(Joint)/P(Pose)/S(Station_relative_pose) vel(0.0~1.0) acc(0.0~1.0) point_name
            # run("A S PTP J 0.1 0.1 test1")
            # run("A S PTP J 0.1 0.1 test2")

            # run("A S PTP P 0.03 0.03 test1")
            # run("A S PTP P 0.03 0.03 test2")

            # run("A S LIN P 0.4 0.4 test1")
            # run("A S LIN P 0.4 0.4 test2")

            # # 延时 3s
            # run("W 3.0")

            # # 夹爪控制 G pgi_pos(0-1000) rgi_pos(0-1000) rel_angle abs_angle
            # run("g 100 100 100 100 100 100")
            # run("G 0 -1 0 99999999")
            # run("G 1000 -1 0 99999999")
            # run("G -1 0 0 99999999")
            # run("G -1 1000 0 99999999")
            # run("G -1 0 360 99999999")
            # run("G -1 500 360 99999999")
            # run("G -1 1000 360 99999999")
            # run("G -1 500 -360 99999999")
            # run("G -1 0 -360 99999999")
            # run("G -1 0 0 1000")
            # run("G -1 0 0 500")
            # run("G -1 0 0 0")
            # run("G -1 0 0 500")
            # run("G -1 0 0 -500")
            # run("G -1 0 0 500")

            # # 夹爪参数设置 g pgi_force(20-100) pgi_velocity(1-100) rgi_force(20-100) rgi_velocity(1-100) rgi_torque(20-100) rgi_speed(1-100)
            # run("g 20 20 20 20 20 20")
            # run("G 0 1000 0 99999999")
            # run("g 100 100 100 100 100 100")
            # run("G 1000 0 1000 99999999")
            # run("g -1 50 -1 50 -1 -1")
            # run("G 0 1000 0 99999999")

            # # 执行动作组 scripts/stationdata/table.json/grab_block_to_pgi
            # # @后方字符将被替换至动作组内每个动作点名称中：block@_high ---> block4_high
            # # 夹爪状态判定："G -1 0 0 99999999%idle clamping",要求执行完后PGI和RGI的状态必须分别为idle和clamping
            # run("grab_block_to_pgi@4")

            # # ～～～～～～～自行完成～～～～～～～：
            # # 依次将四个方块送至pgi夹爪夹紧，随后取回
            # run("take_back_block@4")
            # run("grab_block_to_pgi@3")
            # run("take_back_block@3")
            # run("grab_block_to_pgi@2")
            # run("take_back_block@2")
            # run("grab_block_to_pgi@1")
            # run("take_back_block@1")
            # ～～～～～～～～～～～～～～～～
            
            # run("R L auto_balance")
            # run("F J auto_balance")

            # run('relocation')
            # run('g -1 20 -1 -1 -1 -1')
            # run("take_sample")
            # run("put_sample")
            # run('g -1 100 -1 -1 -1 -1')
            # def calib(name):
            #     run(f"A L LIN J 0.1 0.1 {name}")
            #     continue_or_not()
            # # calib('take_out_safe')
            # calib('ready_put_take')


            # run('R L fume_hood')
            # run('test')
            # run("Screw_take_bottle@1")
            # run("Exp_take_pipe")
            # run("Exp_place_pipe")
            break
  
    except rospy.ROSInterruptException:
        is_succ = False
    if not is_succ:
        print(json.dumps({"code": 500, "spe_msg_sign": "failed"}))
    else:
        print(json.dumps({"code": 200, "spe_msg_sign": "success"}))