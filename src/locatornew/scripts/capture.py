#!/usr/bin/env python3
import numpy as np
import time
import json
import rospy
import time
import pyrealsense2 as rs2  # RealSense相机SDK，用于图像采集
import cv2  # 图像处理库，用于图像保存
from tqdm import tqdm  # 进度条库（当前未使用，预留用于批量任务）
import sys
import csv  # 用于保存机械臂位姿数据到CSV格式
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import math
from scipy.spatial.transform import Rotation as R  # 旋转矩阵处理库
from icecream import ic  # 调试工具，用于打印变量详情
from robot_controller import Robot  # 自定义机械臂控制类（封装机械臂运动API）
from calculate_rotate import *  # 自定义坐标变换/旋转计算模块


# ------------------------------ Camera类：RealSense相机控制 ------------------------------
class Camera:
    """RealSense相机控制类，实现相机初始化、图像捕获、图像保存功能"""
    
    def __init__(self):
        """初始化相机，配置RGB流（1920x1080分辨率，30fps，BGR格式）"""
        try:
            self.pipe = rs2.pipeline()  # 创建相机流水线（管理相机数据流）
            config = rs2.config()  # 创建配置对象
            # 启用RGB流：分辨率1920x1080，格式BGR8（OpenCV兼容），帧率30
            config.enable_stream(rs2.stream.color, 1920, 1080, rs2.format.bgr8, 30)
            self.profile = self.pipe.start(config)  # 启动相机并应用配置
        except Exception as e:
            print(f"相机启动失败: {e}")  # 捕获启动异常（如相机未连接、权限不足）
            self.pipe = None  # 标记相机未初始化成功

    def saveFrameTo(self, path):
        """
        捕获一帧RGB图像并保存到指定路径
        :param path: 图像保存路径（如"./img_take/0.png"）
        """
        if self.pipe is None:
            print("相机未成功启动，无法保存帧。")
            return
        try:
            frames = self.pipe.wait_for_frames()  # 等待相机帧数据（阻塞，直到获取帧）
            # 提取RGB帧并转换为numpy数组（OpenCV可处理格式）
            img = np.asarray(frames.get_color_frame().get_data())
            cv2.imwrite(path, img)  # 保存图像到指定路径
            print(f"帧已保存到 {path}")
            return img
        except Exception as e:
            print(f"保存帧时出错: {e}")  # 捕获保存异常（如路径不存在、磁盘满）

    def getRGBFrame(self):
        """
        获取原始RGB帧（不保存，用于实时处理）
        :return: 成功返回rs2.color_frame对象，失败返回None
        """
        if self.pipe is None:
            print("相机未成功启动，无法获取帧。")
            return None
        try:
            # stime=time.time()
            # 等待帧并提取RGB帧（不转换为numpy数组，保留原始格式）
            frame = self.pipe.wait_for_frames().get_color_frame()
            # print(f'取图时间 {(time.time()-stime)*1000:.1f} ms')
            return frame
        except Exception as e:
            print(f"获取帧时出错: {e}")
            return None

    def __del__(self):
        """析构函数：对象销毁时停止相机，释放硬件资源"""
        if self.pipe is not None:
            self.pipe.stop()  # 停止相机流水线，避免资源泄漏


# 实例化Camera对象（全局单例，供MoveAndCapture类使用）
camera = Camera()


# ------------------------------ MoveAndCapture类：机械臂运动+图像捕获协同控制 ------------------------------
class MoveAndCapture:
    """机械臂运动与相机捕获协同类，根据配置文件控制机械臂移动到目标位姿并拍照，记录位姿数据"""
    
    def __init__(self, config):
        """
        初始化协同控制对象
        :param bTt: 基础坐标系到目标坐标系的变换矩阵（4x4，用于坐标转换）
        """
        # 1. 创建图像保存目录（./img_take），用于存储拍摄的图像和位姿数据
        file_path = "./img_take"
        self.file_path = file_path
        if not os.path.exists(file_path):
            os.mkdir(file_path)  # 目录不存在则创建

        # 2. 清空目录中之前的文件（避免旧数据干扰当前任务）
        for file in os.listdir(file_path):
            os.remove(f'{file_path}/{file}')  # 删除目录下所有文件

        # 3. 初始化位姿数据文件（pose.txt），用于记录每个拍摄点的机械臂位姿
        fpose = open(f'{file_path}/pose.txt', 'w')
        self.fpose = fpose  # 保存文件句柄，供后续写入
        fpose.write('id,px,py,pz,rx,ry,rz\r\n')  # 写入CSV表头：id（拍摄序号）+ 位姿（x/y/z/rx/ry/rz）
        self.csvwriter = csv.writer(fpose)  # 创建CSV写入器，方便按行写入数据

        # 4. 初始化机械臂控制对象（通过Robot类封装机械臂运动指令）
        self.robot = Robot()
        # 5. 关联全局相机对象（复用之前初始化的RealSense相机）
        self.camera = camera

        # 6. 存储机械臂位姿变化量（用于后续计算运动精度，如均值、标准差）
        self.allDeltaPose = []
        # 7. 读取任务配置文件（拍摄范围、任务类型等）
        self.config=config
        if self.config.get('matrix') is None:
            self.config['matrix'] = 'A1'
        if self.config.get('moveP_vel') is None:
            self.config['moveP_vel']=0.1
        if self.config.get('moveP_acc') is None:
            self.config['moveP_acc']=0.1
        self.publish_tf=None
        # 读取相机内参和畸变参数（用于坐标变换）
        self.gTc, self.camera_matrix, self.dist_coeffs = self.readIntrinsic()
    def read_gTc(self):
        # 获取相机内参
        gTc = np.loadtxt(f"matrix/{self.config['arm']}_gTc.txt", delimiter=',')
        return gTc
    def readCameraMatrix(self):
        camera_matrix = np.loadtxt(f"matrix/{self.config['arm']}_camera_matrix.txt", delimiter=',')
        dist_coeffs = np.loadtxt(f"matrix/{self.config['arm']}_dist_coeffs.txt", delimiter=',')
        return camera_matrix, dist_coeffs
    # 读取相机内参
    def readIntrinsic(self):
        gTc = self.read_gTc()
        camera_matrix, dist_coeffs = self.readCameraMatrix()
        return gTc, camera_matrix, dist_coeffs
    def readConfig(self, file):
        """
        读取任务配置文件（JSON格式），解析拍摄参数
        :param file: 配置文件路径
        """
        with open(file, 'r') as f:
            config = json.load(f)  # 加载JSON配置
        print('~~~~~~~~拍摄参数~~~~~~')
        ic(file)  # 调试打印：配置文件路径
        ic(config)  # 调试打印：完整配置内容（如任务类型、拍摄范围、点数）
        self.config = config  # 保存配置到类属性

        # 兼容配置：若配置中无"matrix"字段（坐标系标识），默认设为"A1"
        if self.config.get('matrix') is None:
            self.config['matrix'] = 'A1'
        if self.config.get('moveP_vel') is None:
            self.config['moveP_vel']=0.1
        if self.config.get('moveP_acc') is None:
            self.config['moveP_acc']=0.1

    def generate_all_xyz(self, config):
        """
        根据任务类型（标定/calib 或 定位/loc）生成所有目标点的XYZ坐标（目标坐标系下）
        :param config: 任务配置字典
        :return: allXYZ - 目标点列表（每个元素为[x, y, z]，单位：毫米）
        """
        config = self.config
        task = config['task']
        # 从配置中提取拍摄范围（目标坐标系下，单位：毫米）
        minX = config[task]['minX']  # X轴最小范围
        maxX = config[task]['maxX']  # X轴最大范围
        minY = config[task]['minY']  # Y轴最小范围
        maxY = config[task]['maxY']  # Y轴最大范围
        minZ = config[task]['minZ']  # Z轴最小范围
        maxZ = config[task]['maxZ']  # Z轴最大范围

        # 提取任务参数（标定任务需固定点数，定位任务需随机点数）
        pointNumX = config['calib']['pointNumX']  # 标定任务X轴点数
        pointNumY = config['calib']['pointNumY']  # 标定任务Y轴点数
        pointNumZ = config['calib']['pointNumZ']  # 标定任务Z轴点数
        allNumber = config['multiloc']['allNumber']    # 定位任务总点数

        # 校验任务类型（仅支持calib/loc）
        assert task in ['calib', 'multiloc'], "任务类型错误，仅支持'calib'或'multiloc'"
        allXYZ = []

        if task == 'calib':
            # 标定任务：生成均匀分布的网格点（X→Y→Z顺序遍历）
            allX = np.linspace(minX, maxX, pointNumX)  # X轴均匀取点
            allY = np.linspace(minY, maxY, pointNumY)  # Y轴均匀取点
            allZ = np.linspace(minZ, maxZ, pointNumZ)  # Z轴均匀取点
            # 三维网格遍历：先Z轴，再X轴，最后Y轴（确保覆盖整个拍摄空间）
            for z in allZ:
                for x in allX:
                    for y in allY:
                        allXYZ.append([x, y, z])
        else:  # task == 'multiloc'
            # 定位任务：生成随机分布的点（覆盖指定范围）
            for i in range(allNumber):
                allXYZ.append([
                    np.random.uniform(minX, maxX),  # X轴随机值
                    np.random.uniform(minY, maxY),  # Y轴随机值
                    np.random.uniform(minZ, maxZ),  # Z轴随机值
                ])
        return allXYZ

    def savePhotoAndPose(self, nums=0, oneloc=False):
        """
        拍摄图像并记录机械臂位姿（含运动精度计算）
        :param nums: 拍摄序号（用于图像命名和位姿记录ID）
        """
        # 1. 获取机械臂运动前的位姿（基础坐标系下，单位：米/弧度）
        initial_state = self.robot.getPoseBase()
        # 2. 拍摄图像并保存（命名格式：序号.png，如0.png）
        image_name='oneloc.png' if oneloc else f'multiloc_{nums}.png'
        self.camera.saveFrameTo(f'{self.file_path}/{image_name}')
        # 3. 获取机械臂运动后的位姿（用于计算运动前后的偏差）
        final_state = self.robot.getPoseBase()

        # 4. 计算位姿变化量（转换为毫米单位，方便观察）
        delta_state = (np.array(final_state) - np.array(initial_state)) * 1000
        self.allDeltaPose.append(delta_state)  # 保存变化量，用于后续统计

        # 5. 计算运动前后的平均位姿（作为当前拍摄点的最终位姿）
        current_state = list(np.mean([initial_state, final_state], axis=0))
        # ic(current_state)  # 调试打印：当前拍摄点的平均位姿
        # 6. 将拍摄序号和最终位姿写入CSV文件（id, px, py, pz, rx, ry, rz）
        self.csvwriter.writerow([nums] + list(final_state))
        print(f'成功保存图片"./img_take/{image_name}与位姿')

    def moveAndTakePhoto(self, nums, targetXYZ_tTc):
        """
        核心函数：计算机械臂目标位姿→控制机械臂移动→拍照→记录位姿
        :param nums: 拍摄序号
        :param targetXYZ_tTc: 目标坐标系下的XYZ坐标（单位：毫米）
        """
        # ic(nums, targetXYZ_tTc)  # 调试打印：拍摄序号和目标点（目标坐标系下）

        # 1. 坐标变换：将目标坐标系下的点转换为机械臂基础坐标系下的点+姿态
        # （调用自定义函数transform_xyz_tTc_to_bTc，输入：目标点、bTt变换矩阵、坐标系标识）
        target_bTg = transform_xyz_tTc_to_bTg(targetXYZ_tTc, self.bTt, self.gTc, self.config['matrix'], self.camera_matrix, self.dist_coeffs)
        # if self.publish_tf is not None:
        #     self.publish_tf(target_bTg, 'b', 'g')
        # 7. 从齐次变换矩阵中提取平移向量和旋转向量（格式适配机械臂控制）
        # 提取平移向量：齐次矩阵上右3x1部分，展平为1D数组
        point = target_bTg[:3, 3].flatten()/1000
        # 提取旋转矩阵并转换为旋转向量（Rodrigues向量）：
        # 'xyz' 表示旋转顺序：绕X轴（Roll）→ 绕Y轴（Pitch）→ 绕Z轴（Yaw），与 xyzrpyToPose 一致
        rpy_rad = R.from_matrix(target_bTg[:3, :3]).as_euler('xyz')  
        # 转为角度制（因为 xyzrpyToPose 输入的 R/P/Y 是角度制）
        rpy_deg = np.rad2deg(rpy_rad)  
        # 构造机械臂目标位姿：[x, y, z（米）, Roll, Pitch, Yaw（角度）]
        pose = [point[0], point[1], point[2], rpy_deg[0], rpy_deg[1], rpy_deg[2]]
        print(f"第{nums}个目标点：{point}, rpy_deg:{rpy_deg}")
        # return
        # 2. 安全校验：Z轴坐标范围（避免机械臂碰撞，假设安全范围：-0.2m ~ 1m）
        z = point[2]
        assert -.2 <= z <= 1, f"Z轴坐标超出安全范围：{z}m（安全范围：-0.2~1m）"

        # 3. 构造机械臂目标位姿（[x, y, z, rx, ry, rz]，单位：米/弧度）
        pose = [point[0], point[1], point[2], rpy_deg[0], rpy_deg[1], rpy_deg[2]]
        # 4. 控制机械臂移动到目标位姿（速度0.2m/s，加速度0.1m/s²，调用Robot类的moveP函数）
        self.robot.moveP(self.config['moveP_vel'], self.config['moveP_acc'], pose)

        # 5. 等待机械臂稳定（0.1秒延迟，避免运动未结束就拍照）
        time.sleep(.1)
        # 6. 拍照并记录位姿
        self.savePhotoAndPose(nums)

    def moveAndCaptureAll(self, bTt):
        """
        批量执行拍摄任务：根据任务类型（oneloc/calib/loc）控制机械臂移动并拍照
        是整个类的入口函数，串联所有流程
        """
        print('~~~~~~~~开始拍摄~~~~~~~~')
        self.bTt=bTt
        config = self.config
        task = config['task']  # 获取任务类型（oneloc：单次拍摄；calib：标定；loc：定位）

        if task == 'oneloc':  # 单次拍摄任务（如初始定位、单帧校准）
            print(f'单次拍摄 task={task}')
            self.savePhotoAndPose(0)  # 直接拍照（序号0），无需机械臂移动
        else:  # 多次拍摄任务（calib或loc）
            print(f'多次拍摄 task={task}')
            allXYZ = self.generate_all_xyz(config)  # 生成所有目标点
            # 遍历所有目标点，依次移动并拍照
            for nums in range(len(allXYZ)):
                targetXYZ_tTc = allXYZ[nums]  # 当前目标点（目标坐标系下）
                self.moveAndTakePhoto(nums, targetXYZ_tTc)

        # 任务结束后：统计机械臂运动精度（位姿变化量的均值和标准差）
        mean = np.mean(self.allDeltaPose, axis=0)  # 变化量均值（越小越稳定）
        std = np.std(self.allDeltaPose, axis=0)    # 变化量标准差（越小越精确）
        # ic(mean)  # 调试打印：运动精度均值
        # ic(std)   # 调试打印：运动精度标准差

        # 将运动精度统计结果写入日志文件（capture-log.txt）
        with open(f'{self.file_path}/capture-log.txt', 'w') as f:
            f.write(f'mean delta: {mean}\n')  # 均值：各轴平均偏差（毫米）
            f.write(f'std delta: {std}\n')    # 标准差：各轴偏差波动（毫米）

        # 释放资源：关闭位姿文件、断开机械臂连接
        self.fpose.close()
        self.robot.close()
        print('~~~~~~~~结束拍摄~~~~~~~~')


# ------------------------------ 主函数：任务入口 ------------------------------
if __name__ == '__main__':
    # 初始化ROS节点（用于与机械臂/其他模块通信，匿名=True避免节点名冲突）
    rospy.init_node('capture_node', anonymous=True)
    
    # 加载基础坐标系到目标坐标系的变换矩阵（从txt文件读取，逗号分隔）
    # 注：该矩阵通常来自前期标定（如oneloc任务的输出），是坐标变换的核心依据
    bTt = np.loadtxt('matrix/oneloc_bTt.txt', delimiter=',')
    
    # 校验命令行参数：需传入1个配置文件路径（如python capture.py capture-config/oneloc.json）
    assert len(sys.argv) == 2, "参数错误！用法：python capture.py <配置文件路径>"
    configFile = sys.argv[1]  # 获取命令行传入的配置文件路径

    # 初始化MoveAndCapture对象并执行批量拍摄任务
    moveAndCapture = MoveAndCapture(configFile, bTt)
    moveAndCapture.moveAndCaptureAll()