#!/usr/bin/env python3
import cv2 as cv
import re
import numpy as np
from icecream import ic
import time
import threading
import pyrealsense2
from scipy.spatial.transform import Rotation as R
import os
import json
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from itertools import combinations
import rospy
import tf2_ros
import geometry_msgs.msg
# from matplotlib import pyplot as plt
from matrix_utils import *
import scipy.stats as stats
from statsmodels.stats.stattools import durbin_watson

np.set_printoptions(precision=6, suppress=True)
# np.random.seed(0)

def ic(*args):
    pass

def durbinWatsonTest(data):
    dw = durbin_watson(data)
    ic(dw)

def shapiro_test(data):
    stat, p = shapiro(data)
    print()
    print(f"Shapiro-Wilk Test: W = {stat:.4f}, p-value = {p:.4f}")
    if p > 0.05:
        print("结论: 数据服从正态分布 (无法拒绝原假设)")
    else:
        print("结论: 数据不服从正态分布 (拒绝原假设)")


def readConfig(file):
    with open(file, 'r') as f:
        config = json.load(f)
    ic(config)
    return config

def parse_pose_txt(txt_path):
    fpose = open(txt_path, 'r')
    ftxt = fpose.readlines()
    if ftxt[0].startswith('id,'):
        ftxt.pop(0)
    poses = {}
    for i in range(0, len(ftxt)):
        fline = ftxt[i]
        data = [float(x) for x in fline.split(',')]
        # assert data[0] == float(i + 1)

        xyz = data[1:4]
        xyz = [1000 * x for x in xyz]
        quaternion = data[4:]
        poses[int(data[0])] = {'xyz': xyz, 'quaternion': quaternion}
    return poses


def getAllImage(imageAndPoseDirectory):
    print('getAllImage:', imageAndPoseDirectory)
    # 获取目录下所有文件
    files = os.listdir(imageAndPoseDirectory)
    
    # 步骤1：先筛选「.png后缀 + 文件名含数字」的文件
    def has_number(filename):
        """判断文件名（不含后缀）是否包含数字"""
        name_without_ext = filename.split('.')[0]  # 去掉 .png 后缀
        return re.search(r'\d+', name_without_ext) is not None  # 存在数字则返回True
    
    # 筛选逻辑：同时满足「.png后缀」和「含数字」
    imageFiles = [
        file for file in files 
        if file.endswith('.png') and has_number(file)
    ]
    
    # 步骤2：若筛选后无文件，打印警告并返回空列表
    if not imageFiles:
        print(f"警告：目录 {imageAndPoseDirectory} 中无含数字的 .png 图片")
        return []
    
    # 步骤3：按文件名中的数字排序（保持原排序逻辑，仅对筛选后的文件生效）
    def extract_number(filename):
        name_without_ext = filename.split('.')[0]
        match = re.search(r'\d+', name_without_ext)
        return int(match.group()) if match else 0  # 筛选后已确保有数字，match不会为None
    
    imageFiles.sort(key=extract_number)
    
    # 步骤4：拼接完整路径
    imageFiles = [os.path.join(imageAndPoseDirectory, file) for file in imageFiles]
    print(f"筛选并排序后，有效图片数量：{len(imageFiles)}，路径列表：{imageFiles}")
    return imageFiles

def trans_pose_to_bTg(pose):
    """
    将位姿转换为基座到工具的变换矩阵bTg
    支持两种输入格式：
    1. 字典类型: {'xyz': [x, y, z], 'quaternion': [qx, qy, qz, qw]}
    2. 向量类型: [x, y, z, qx, qy, qz, qw]（长度为7的列表或数组）
    """
    # 解析位姿数据（区分字典和向量类型）
    if isinstance(pose, dict):
        # 处理字典类型（原始逻辑）
        tVec = pose['xyz']         # 平移分量 [x, y, z]
        qVec = pose['quaternion']  # 四元数 [qx, qy, qz, qw]
    elif isinstance(pose, (list, np.ndarray)):
        # 处理向量类型（假设顺序为[x, y, z, qx, qy, qz, qw]）
        if len(pose) != 7:
            raise ValueError(f"向量类型pose必须包含7个元素，实际为{len(pose)}个")
        
        # 提取平移分量（前3个元素）和四元数（后4个元素）
        tVec = [v*1000 for v in pose[:3]]
        qVec = pose[3:7]
    else:
        # 不支持的类型
        raise TypeError(f"不支持的pose类型：{type(pose)}，仅支持dict、list或np.ndarray")
    
    # 转换四元数到旋转向量
    rVec = quaternionToRotvec(qVec)
    
    # 旋转向量+平移向量 → 4x4变换矩阵
    bTg = rtToMatrix(rVec, tVec)
    return bTg
def read_bTg(imageAndPoseDirectory, imageIndex):
    allPose = parse_pose_txt(imageAndPoseDirectory + '/pose.txt')
    return trans_pose_to_bTg(allPose[imageIndex])

def read_all_bTg(imageAndPoseDirectory, allImageFile):
    all_bTg = []
    print('read all bTg:', allImageFile)
    
    for imageFile in allImageFile:
        # 1. 从图片路径中提取“文件名前缀”（如 "img_take/multiloc_1.png" → "multiloc_1"）
        # 步骤：先取路径最后一部分（文件名）→ 再去掉后缀 .png
        filename_without_ext = os.path.basename(imageFile).split('.')[0]
        # print(f"处理图片：{imageFile} → 文件名前缀：{filename_without_ext}")
        
        # 2. 提取文件名中的数字作为 index（核心优化）
        # 用正则表达式匹配“一个或多个连续数字”，若存在则转整数，否则处理特殊情况
        num_match = re.search(r'\d+', filename_without_ext)
        if num_match:
            # 场景1：含数字（如 "multiloc_1" → 提取 1，"multiloc_0" → 提取 0）
            index = int(num_match.group())
            # 3. 读取对应 index 的 bTg 数据（沿用你的 read_bTg 函数）
            temp_bTg = read_bTg(imageAndPoseDirectory, index)
            if temp_bTg is not None:  # 若 read_bTg 可能返回 None，需过滤无效数据
                all_bTg.append(temp_bTg)
            else:
                rospy.logwarn(f"未读取到{imageFile}对应的 bTg 数据，跳过")
        else:
            continue
    
    return all_bTg

def read_all_image_and_bTg(imageAndPoseDirectory, pose_txt_name="pose.txt"):
    """
    整合函数：读取目录中所有「含数字的.png图片」，按数字排序，匹配对应bTg矩阵，打包返回
    Args:
        imageAndPoseDirectory: 图片、pose.txt文件所在目录（如 "img_take/"）
        pose_txt_name: pose文件名称（默认 "pose.txt"，可按需修改）
    Returns:
        list: 元素为 tuple(image_file_path, bTg_matrix)，按图片文件名数字升序排列；
              若无有效图片/无对应bTg，返回空列表
    """
    # -------------------------- 步骤1：筛选+排序含数字的.png图片（复用getAllImage逻辑） --------------------------
    def has_number(filename):
        """辅助函数：判断文件名（不含后缀）是否包含数字"""
        name_without_ext = filename.split('.')[0]
        return re.search(r'\d+', name_without_ext) is not None

    # 1.1 筛选目录中「.png后缀 + 含数字」的文件
    all_files = os.listdir(imageAndPoseDirectory)
    valid_png_files = [
        f for f in all_files 
        if f.endswith('.png') and has_number(f)
    ]

    # 1.2 处理无有效图片的情况
    if not valid_png_files:
        rospy.logwarn(f"目录 {imageAndPoseDirectory} 中无「含数字的.png图片」，返回空列表")
        return []

    # 1.3 按文件名中的数字升序排序
    def extract_number(filename):
        """辅助函数：提取文件名中的数字（用于排序）"""
        name_without_ext = filename.split('.')[0]
        num_match = re.search(r'\d+', name_without_ext)
        return int(num_match.group()) if num_match else 0  # 筛选后num_match必不为None

    valid_png_files.sort(key=extract_number)

    # 1.4 拼接图片完整路径（如 "img_take/multiloc_0.png"）
    sorted_image_paths = [
        os.path.join(imageAndPoseDirectory, png_file) 
        for png_file in valid_png_files
    ]
    rospy.loginfo(f"筛选排序后，有效图片数量：{len(sorted_image_paths)}")


    # -------------------------- 步骤2：读取pose.txt，获取所有原始位姿（复用read_bTg依赖的parse_pose_txt） --------------------------
    # 拼接pose.txt完整路径
    pose_txt_path = os.path.join(imageAndPoseDirectory, pose_txt_name)
    # 读取pose.txt并解析为原始位姿列表（需确保你的parse_pose_txt函数能正常工作）
    try:
        all_raw_poses = parse_pose_txt(pose_txt_path)
        rospy.loginfo(f"成功读取pose.txt，共 {len(all_raw_poses)} 个原始位姿")
    except Exception as e:
        rospy.logerr(f"读取/解析pose.txt（{pose_txt_path}）失败：{str(e)}，返回空列表")
        return []


    # -------------------------- 步骤3：匹配图片与对应的bTg矩阵（复用read_bTg逻辑） --------------------------
    image_and_bTg_list = []

    for image_path in sorted_image_paths:
        # 3.1 从图片路径提取文件名（如 "img_take/multiloc_0.png" → "multiloc_0.png"）
        image_filename = os.path.basename(image_path)
        # 3.2 提取文件名中的数字（作为匹配pose.txt的index）
        name_without_ext = image_filename.split('.')[0]
        num_match = re.search(r'\d+', name_without_ext)
        image_index = int(num_match.group())  # 筛选后num_match必不为None

        # 3.3 校验index是否在pose.txt的有效范围内（避免越界）
        if image_index < 0 or image_index >= len(all_raw_poses):
            rospy.logwarn(f"图片 {image_filename} 的index={image_index} 超出pose.txt范围（0~{len(all_raw_poses)-1}），跳过")
            continue

        # 3.4 解析原始位姿为bTg矩阵（复用trans_pose_to_bTg函数）
        try:
            bTg_matrix = trans_pose_to_bTg(all_raw_poses[image_index])
            if bTg_matrix is None:
                rospy.logwarn(f"图片 {image_filename} 的index={image_index} 对应的位姿无法转为bTg矩阵，跳过")
                continue
        except Exception as e:
            rospy.logerr(f"转换图片 {image_filename} 的位姿为bTg时出错：{str(e)}，跳过")
            continue

        # 3.5 匹配成功，添加到结果列表
        image_and_bTg_list.append( (image_path, bTg_matrix) )
        rospy.logdebug(f"图片 {image_filename} 匹配bTg成功，index={image_index}")


    # -------------------------- 步骤4：返回最终结果 --------------------------
    rospy.loginfo(f"最终有效（图片路径, bTg）对数量：{len(image_and_bTg_list)}")
    return image_and_bTg_list

def showUndistortedImage(image_file, camera_matrix, dist_coeffs):
    image = cv.imread(image_file)
    image_undistorted = cv.undistort(image, camera_matrix, dist_coeffs)
    cv.imshow('undistorted', image_undistorted)
    cv.waitKey(0)


def generateRectangle(minX, maxX, minY, maxY):
    return np.array([[minX, minY, 0], [maxX, minY, 0], [maxX, maxY, 0], [minX, maxY, 0]])


# 输入是 n*3 的矩阵，输出 4 个点的 2d 坐标
def movePointsToRectangleCenter(object_points):
    allX = object_points[:, 0]
    allY = object_points[:, 1]
    allZ = object_points[:, 2]

    # 保证 z 轴都是 0
    assert np.all(allZ == 0)

    # 计算最小外接矩形
    minX = np.min(allX)
    maxX = np.max(allX)
    minY = np.min(allY)
    maxY = np.max(allY)

    center = np.array([(minX + maxX) / 2, (minY + maxY) / 2, 0])
    newObjectPoints = object_points - center
    rectanglePoints = generateRectangle(minX, maxX, minY, maxY)
    return newObjectPoints, rectanglePoints, center


class Pattern:
    def __init__(self, config):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.current_tf = {}
        self.last_tf={}
        self.alpha=0.3
        self.tf_timer_running = False
        self.tf_timer_thread = None
        self.start_tf_timer()  # 启动定时器

        self.config=config
        task = config['task']
        self.task = task
        self.type = config['type']
        self.imageAndPoseDirectory = config['imageAndPoseDirectory']
        self.poseRelativeFile = config['poseRelativeFile']
        self.calibResultFile = config['calibResultFile']

        gTc, camera_matrix, dist_coeffs = self.readIntrinsic()
        self.gTc = gTc
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.publish_tf(gTc,'g','c')
        
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
    
    def start_tf_timer(self):
        """启动独立的TF发布定时器线程"""
        if not self.tf_timer_running:
            self.tf_timer_running = True
            self.tf_timer_thread = threading.Thread(target=self.tf_timer_loop, daemon=True)
            self.tf_timer_thread.start()
            rospy.loginfo("TF发布定时器已启动")
    def tf_timer_loop(self):
        """定时器循环，使用alpha系数滤波器发布平滑的TF变换"""
        rate = 30  # 30Hz发布频率
        while self.tf_timer_running and not rospy.is_shutdown():
            for key in list(self.current_tf.keys()):
                new_transform = self.current_tf[key]
                
                # 应用alpha系数滤波
                filtered_transform = self.alpha_filter(key, new_transform)
                
                # 更新时间戳并发布
                filtered_transform.header.stamp = rospy.Time.now()
                self.tf_broadcaster.sendTransform(filtered_transform)
            
            time.sleep(1.0 / rate)

    def alpha_filter(self, key, new_transform):
        """
        指数移动平均滤波：filtered = alpha * new + (1 - alpha) * last
        :param key: TF变换的标识键
        :param new_transform: 新的原始TF变换
        :return: 滤波后的TF变换
        """
        # 若为首次处理该TF，直接使用新值作为初始值
        if key not in self.last_tf:
            self.last_tf[key] = new_transform
            return new_transform
        
        # 获取上一次的滤波结果
        last = self.last_tf[key]
        filtered = geometry_msgs.msg.TransformStamped()
        filtered.header = new_transform.header
        filtered.child_frame_id = new_transform.child_frame_id
        
        # 平移分量滤波：alpha×新值 + (1-alpha)×旧值
        filtered.transform.translation.x = self.alpha * new_transform.transform.translation.x + \
                                        (1 - self.alpha) * last.transform.translation.x
        filtered.transform.translation.y = self.alpha * new_transform.transform.translation.y + \
                                        (1 - self.alpha) * last.transform.translation.y
        filtered.transform.translation.z = self.alpha * new_transform.transform.translation.z + \
                                        (1 - self.alpha) * last.transform.translation.z
        
        # 旋转分量（四元数）滤波：同样使用指数加权平均，最后归一化
        filtered.transform.rotation.x = self.alpha * new_transform.transform.rotation.x + \
                                    (1 - self.alpha) * last.transform.rotation.x
        filtered.transform.rotation.y = self.alpha * new_transform.transform.rotation.y + \
                                    (1 - self.alpha) * last.transform.rotation.y
        filtered.transform.rotation.z = self.alpha * new_transform.transform.rotation.z + \
                                    (1 - self.alpha) * last.transform.rotation.z
        filtered.transform.rotation.w = self.alpha * new_transform.transform.rotation.w + \
                                    (1 - self.alpha) * last.transform.rotation.w
        
        # 归一化四元数（确保旋转分量合法性）
        norm = np.sqrt(
            filtered.transform.rotation.x**2 +
            filtered.transform.rotation.y**2 +
            filtered.transform.rotation.z**2 +
            filtered.transform.rotation.w**2
        )
        if norm > 0:
            filtered.transform.rotation.x /= norm
            filtered.transform.rotation.y /= norm
            filtered.transform.rotation.z /= norm
            filtered.transform.rotation.w /= norm
        
        # 更新上一次的滤波结果
        self.last_tf[key] = filtered
        return filtered
    def remove_tf(self, key):
        """移除不需要发布的TF变换"""
        if key in self.current_tf:
            del self.current_tf[key]
            rospy.loginfo(f"已停止发布TF变换: {key}")
    def shutdown(self):
        """关闭定时器线程"""
        self.tf_timer_running = False
        if self.tf_timer_thread:
            self.tf_timer_thread.join()
        rospy.loginfo("TF发布定时器已关闭")
    def __del__(self):
        self.shutdown()
        
    def calculateOneloc(self, object_points, image_points, bTg):
        # 计算相机坐标系下 target 的位姿
        cTt = self.calculate_cTt(object_points, image_points, self.camera_matrix, self.dist_coeffs)
        gTc = self.gTc

        # 计算 target 坐标系下的相机位姿
        tTc = np.linalg.inv(cTt)
        bTc = np.dot(bTg, gTc)
        bTt = np.dot(bTc, cTt)
        # self.publish_tf(bTg,'b','g')
        # self.publish_tf(gTc,'g','c')
        # self.publish_tf(cTt,'c','t')
        # self.publish_tf(bTc,'b','c')
        ic(bTt)
        np.savetxt('matrix/oneloc_bTt.txt', bTt, fmt='%.6f', delimiter=',')
        return bTt
    def calculate_bTt_std(self, imageLength, all_bTt):
        all_bTt = np.array(all_bTt)
        for i,bTt in enumerate(all_bTt):
            print(f'bTt({i}) {bTt[0,3]:.3f} {bTt[1,3]:.3f} {bTt[2,3]:.3f}')
        # 检查 all_bTt 是否为空
        if all_bTt.size == 0:
            print("Warning: all_bTt is empty. Skipping calculations.")
            return None
        # 检查 all_bTt 长度是否为 1
        if len(all_bTt) == 1:
            print("Warning: all_bTt has only one element. Skipping calculations that require multiple elements.")
            # 可以选择只计算 all_bTt 的均值
            ic(np.mean(all_bTt, axis=0))
            return None

        # 第奇数个的变换乘上第偶数个的逆变换，计算差值相对于 gt=0 的误差
        diffOddEven = [np.dot(all_bTt[i], np.linalg.inv(all_bTt[i + 1])) for i in range(0, len(all_bTt) - 1, 2)]
        print(f'diffOddEven={len(diffOddEven)} all_bTt={len(all_bTt)}')
        # diffOddEven = [all_bTt[i] - all_bTt[i + 1] for i in range(0, len(all_bTt) - 1, 2)]
        # 检查 diffOddEven 是否为空
        if len(diffOddEven) == 0:
            print("Warning: diffOddEven is empty. Skipping calculations.")
            return
        # ic(np.mean(all_bTt, axis=0))
        # ic(np.std(all_bTt, axis=0))
        # ic(np.sqrt(np.mean(np.square(all_bTt), axis=0)))

        # ic(np.mean(diffOddEven, axis=0))
        # ic(np.std(diffOddEven, axis=0))
        # ic(np.sqrt(np.mean(np.square(diffOddEven), axis=0)))

        diffOddEven = np.array(diffOddEven)
        ic(len(diffOddEven))

        # Durbin-Watson 检验 diffOddEven 的 x
        durbinWatsonTest(diffOddEven[:, 0, 3])
        durbinWatsonTest(diffOddEven[:, 1, 3])
        durbinWatsonTest(diffOddEven[:, 2, 3])

        durbinWatsonTest(all_bTt[:, 0, 3] - np.mean(all_bTt[:, 0, 3]))
        durbinWatsonTest(all_bTt[:, 1, 3] - np.mean(all_bTt[:, 1, 3]))
        durbinWatsonTest(all_bTt[:, 2, 3] - np.mean(all_bTt[:, 2, 3]))

        all_tq_diff = [matrixToTQ(diff) for diff in diffOddEven]
        all_tq_diff = np.array(all_tq_diff)

        all_tq_bTt = [matrixToTQ(bTt) for bTt in all_bTt]
        all_tq_bTt = np.array(all_tq_bTt)

        mean_diff = np.mean(all_tq_diff, axis=0)
        # ic(mean_diff)

        # for tq_bTt in all_tq_bTt:
        #     print(tq_bTt)

        # print()
        # for tq_diff in all_tq_diff:
        #     print(tq_diff)

        # 计算精度
        std_bTt = np.std(all_tq_bTt, axis=0)
        # ic(std_bTt)

        std_diff = np.std(all_tq_diff, axis=0)
        # ic(std_diff)

        # 计算每个旋转的角度
        all_tq_diff_rotvec = np.array([R.from_quat(tq[3:]).as_rotvec() for tq in all_tq_diff])
        all_tq_diff_angle = np.linalg.norm(all_tq_diff_rotvec, axis=1)
        # 转成角度
        all_tq_diff_angle = np.degrees(all_tq_diff_angle)
        # ic(all_tq_diff_angle)
        # ic(np.mean(all_tq_diff_angle))

        std_err_diff = std_diff / np.sqrt(2 * (len(all_tq_diff) - 1))
        # ic(std_err_diff)

        target = np.array([0, 0, 0, 0, 0, 0, 1])
        newRMSE = np.sqrt(np.mean(np.square(all_tq_diff - target), axis=0))
        # ic(newRMSE)

        # print('A^-1 B 相对于不变换，每个轴的 RMSE 是：', newRMSE)
        translation_rmse = newRMSE[:3]  # 前3项：X/Y/Z轴平移RMSE（单位：mm，根据你的实际单位调整）
        rotation_rmse_rad = newRMSE[3:6]  # 中间3项：X/Y/Z轴旋转RMSE（原始单位：弧度）
        redundant_term = newRMSE[6]  # 最后1项：冗余项（可忽略）

        # 2. 旋转误差单位转换：弧度 → 角度（更直观，1弧度≈57.2958度）
        rotation_rmse_deg = [rad * (180 / np.pi) for rad in rotation_rmse_rad]

        # 3. 直观打印（分模块输出，标注轴、误差值、单位）
        print("=" * 60)
        print("A^-1 B 相对于不变换的误差分析（按类型分类）：")
        print("-" * 60)
        print("【平移误差 RMSE】（单位：mm，越小越好）")
        print(f"  X轴平移误差：{translation_rmse[0]:.6f} mm")
        print(f"  Y轴平移误差：{translation_rmse[1]:.6f} mm")
        print(f"  Z轴平移误差：{translation_rmse[2]:.6f} mm")
        print(f"  平移误差最大值：{max(translation_rmse):.6f} mm")
        print("-" * 60)
        print("【旋转误差 RMSE】（单位：度，越小越好）")
        print(f"  X轴旋转误差（滚转）：{rotation_rmse_deg[0]:.6f} ° （原始：{rotation_rmse_rad[0]:.6f} rad）")
        print(f"  Y轴旋转误差（俯仰）：{rotation_rmse_deg[1]:.6f} ° （原始：{rotation_rmse_rad[1]:.6f} rad）")
        print(f"  Z轴旋转误差（偏航）：{rotation_rmse_deg[2]:.6f} ° （原始：{rotation_rmse_rad[2]:.6f} rad）")
        print(f"  旋转误差最大值：{max(rotation_rmse_deg):.6f} °")
        print("-" * 60)
        # # 创建一个包含三行子图的画布
        # fig, axes = plt.subplots(3, 1, figsize=(8, 12))  # 3 行, 1 列

        # axes[0].hist(all_tq_diff[:, 0], bins=20, color='blue', alpha=0.5)
        # axes[0].set_title(f'x-axis error  mean={mean_diff[0]:.3f} std={std_diff[0]:.3f}')
        # axes[1].hist(all_tq_diff[:, 1], bins=20, color='green', alpha=0.5)
        # axes[1].set_title(f'y-axis error  mean={mean_diff[1]:.3f} std={std_diff[1]:.3f}')
        # axes[2].hist(all_tq_diff[:, 2], bins=20, color='red', alpha=0.5)
        # axes[2].set_title(f'z-axis error  mean={mean_diff[2]:.3f} std={std_diff[2]:.3f}')

        # plt.tight_layout()
        # plt.savefig('data/all_axes_subplots.png')

        # for i in range(len(all_tq_diff[0])):
        #     shapiro_test(all_tq_diff[:, i])
        #     plt.figure()
        #     plt.hist(all_tq_diff[:, i], bins=20, color='blue', alpha=0.5)
        #     plt.savefig(f'data/axis_{i}_hist.png')

        # gt = np.array([0, 0, 0, 0, 0, 0, 1])
        # all_rmse_position = []
        # all_rmse_orientation = []

        # ic(all_tq_diff)
        # rmse_position, rmse_orientation = evaluate_avg(gt, all_tq_diff)
        # ic(rmse_position)
        # ic(rmse_orientation)

        # stderr_rmse_position = rmse_position / np.sqrt(2 * len(all_tq_diff))
        # ic(stderr_rmse_position)

        # for i in range(len(all_tq_diff)):
        #     partDiff = all_tq_diff[i]
        #     (rmse_position, rmse_orientation) = evaluate_avg(gt, partDiff)
        #     all_rmse_position.append(rmse_position)
        #     all_rmse_orientation.append(rmse_orientation)
        # rmse_position = np.mean(all_rmse_position, axis=0)
        # rmse_orientation = np.mean(all_rmse_orientation, axis=0)

    def calculate_cTt(self, object_points, image_points, camera_matrix, dist_coeffs):
        # ic(object_points)
        # ic(image_points)

        assert len(object_points) == len(image_points)
        assert image_points.shape[1] == 2
        assert object_points.shape[1] == 3

        _, pnp_rvecs, pnp_tvecs = cv.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        pnp_rvecs = pnp_rvecs.flatten()
        pnp_tvecs = pnp_tvecs.flatten()
        rot_vec = np.array([np.pi, 0, 0])
        pnp_rvecs = (R.from_rotvec(pnp_rvecs) * R.from_rotvec(rot_vec)).as_rotvec()
        cTt = rtToMatrix(pnp_rvecs, pnp_tvecs)
        return cTt

    def meanMatrix(self, part_bTt):
        # 转成四元数，平均，归一化以后再转回旋转矩阵
        part_quaternions = [matrixToTQ(bTt) for bTt in part_bTt]
        part_quaternions = np.array(part_quaternions)

        mean_quaternion = np.mean(part_quaternions, axis=0)
        mean_t, mean_q = mean_quaternion[:3], mean_quaternion[3:]
        mean_q = mean_q / np.linalg.norm(mean_q)
        mean_quaternion = np.append(mean_t, mean_q)
        mean_matrix = tqToMatrix(mean_quaternion)
        return mean_matrix

    def calculateLoc(self, imageLength, all_cTt, all_bTg, marker_names=None):
        # 准备 bTg, gTc
        gTc = self.read_gTc()

        # 使用矩阵乘法计算 bTt
        all_data_bTt = []
        for bTg, cTt in zip(all_bTg, all_cTt):
            bTt = bTg @ gTc @ cTt
            all_data_bTt.append(bTt)
            if marker_names is not None:
                self.publish_tf(bTt, 'b', 't', marker_names.pop(0))
            else:
                self.publish_tf(bTt, 'b', 't', f'target_marker({len(all_data_bTt)-1})')
        all_data_bTt = np.array(all_data_bTt)

        mean_bTt = self.meanMatrix(all_data_bTt)
        self.calculate_bTt_std(imageLength, all_data_bTt)
        print(f'base 到 target 的齐次矩阵是：\n{mean_bTt}')
        # self.publish_tf(mean_bTt, 'b', 't', 'mean_target')
        mean_bTt[:3, 3] /= 1000
        # print(mean_bTt)
        np.savetxt(self.imageAndPoseDirectory + 'mean_bTt.txt', mean_bTt, fmt='%.6f', delimiter=',')
        return mean_bTt

    def publish_tf(self, matrix, a, b, marker_name=''):#cTt、bTt
        # 从变换矩阵中提取平移和旋转信息
        translation = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]
        rotation = R.from_matrix(rotation_matrix).as_quat()

        # 创建TransformStamped消息
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        if a=='c':
            transform_stamped.header.frame_id = f'{self.config["arm"]}_camera_color_optical_frame'  # 相机坐标系
        elif a=='b':
            transform_stamped.header.frame_id = 'world'  # 机器人基座标
        elif a=='g':
            transform_stamped.header.frame_id = f'{self.config["arm"]}_tool0'  # 机械臂工具坐标
        elif a=='t':
            transform_stamped.header.frame_id = 'target'  # 标定板坐标

        if b=='c':
            transform_stamped.child_frame_id = f'{a}_camera_color_optical_frame'  # 相机坐标系
        elif b=='b':
            transform_stamped.child_frame_id = f'{a}_world'  # 机器人基座标
        elif b=='g':
            transform_stamped.child_frame_id = f'{a}_{self.config["arm"]}_tool0'  # 机械臂工具坐标
        elif b=='t':
            if marker_name=='':
                transform_stamped.child_frame_id = f'target_marker'  # 标定板坐标
            else:
                transform_stamped.child_frame_id = marker_name  # 标定板坐标

        transform_stamped.transform.translation.x = translation[0]/1000
        transform_stamped.transform.translation.y = translation[1]/1000
        transform_stamped.transform.translation.z = translation[2]/1000
        transform_stamped.transform.rotation.x = rotation[0]
        transform_stamped.transform.rotation.y = rotation[1]
        transform_stamped.transform.rotation.z = rotation[2]
        transform_stamped.transform.rotation.w = rotation[3]
        # 发布TF变换
        print(f'发布tf变换:{transform_stamped.header.frame_id}->{transform_stamped.child_frame_id}')
        # 用"父->子"作为键存储变换，相同键会自动更新
        key = f"{transform_stamped.header.frame_id}->{transform_stamped.child_frame_id}"
        self.current_tf[key] = transform_stamped
        return key  # 返回键，方便后续删除
    def calculateCalib(self, imageLength, all_bTg, rvecs, tvecs):
        # 准备 bTg, cTt
        print(f'calculateCalib all_bTg={len(all_bTg)} rvecs={len(rvecs)} tvecs={len(tvecs)}')
        all_cTt = []
        for i in range(imageLength):
            temp_cTt = rtToMatrix(rvecs[i], tvecs[i].flatten())
            all_cTt.append(temp_cTt)

        # ic(all_cTt)

        R_bTg = np.array([bTg[:3, :3] for bTg in all_bTg])
        t_bTg = np.array([bTg[:3, 3] for bTg in all_bTg])
        R_cTt = np.array([cTt[:3, :3] for cTt in all_cTt])
        t_cTt = np.array([cTt[:3, 3] for cTt in all_cTt])

        # ic(R_bTg)
        # ic(t_bTg)
        # ic(R_cTt)
        # ic(t_cTt)

        # 手眼标定，获得 gTc
        R_gTc, t_gTc = cv.calibrateHandEye(R_bTg, t_bTg, R_cTt, t_cTt, method=cv.CALIB_HAND_EYE_TSAI)
        gTc = rotateMatrixTranslationVectorToMatrix(R_gTc, t_gTc.flatten())
        ic(gTc)

        # 计算精度
        # 旋转矩阵转 t, q
        tq_bTg = [matrixToTQ(bTg) for bTg in all_bTg]
        tq_cTt = [matrixToTQ(cTt) for cTt in all_cTt]
        tq_gTc = matrixToTQ(gTc)

        # (rmse_position, rmse_orientation) = evaluate_calibration(tq_bTg, tq_cTt, tq_gTc, 0)
        # ic(rmse_position)
        # ic(rmse_orientation)

        # tT0 is move (60,20,0)
        rotvec0 = np.array([np.pi, 0, 0])
        tvec0 = np.array([60, 20, 0])
        tT0 = rtToMatrix(rotvec0, tvec0)

        # 保存结果
        all_bT0 = []
        all_bTt = []
        for bTg, cTt in zip(all_bTg, all_cTt):
            bT0 = bTg @ gTc @ cTt @ tT0
            all_bT0.append(bT0)
            bTt = bTg @ gTc @ cTt
            all_bTt.append(bTt)

        avg_bT0 = np.mean(all_bT0, axis=0)
        avg_tq_bT0 = matrixToTQ(avg_bT0)

        ic(avg_tq_bT0)
        ic(np.std(all_bT0, axis=0))

        self.calculate_bTt_std(imageLength, all_bTt)
        # 不严格区分 id=0 的 aruco 码的坐标系、target 坐标系
        return gTc


class Charuco(Pattern):
    def __init__(self, config):
        # 父类初始化
        super(Charuco, self).__init__(config)
        assert self.type == 'charuco'

        # 1. Charuco板中嵌入的ArUco标记字典类型
        # 常见值：DICT_5X5_250（5x5像素网格，含250个标记）、DICT_ARUCO_ORIGINAL（原始ArUco字典）
        self.ARUCO_DICT = config['charuco']['ARUCO_DICT']
        # 2. 棋盘格单个方格的物理边长（单位：毫米mm）
        self.SQUARE_LENGTH = config['charuco']['SQUARE_LENGTH']
        # 3. Charuco板中嵌入的ArUco标记的物理边长（单位：毫米mm）
        self.MARKER_LENGTH = config['charuco']['MARKER_LENGTH']
        # 4. 垂直方向（Y轴）的棋盘格数量
        self.SQUARES_VERTICALLY = config['charuco']['SQUARES_VERTICALLY']
        # 5. 水平方向（X轴）的棋盘格数量
        self.SQUARES_HORIZONTALLY = config['charuco']['SQUARES_HORIZONTALLY']
        # 6. 优先识别的ArUco标记ID列表
        self.selectedAruco = config['charuco']['selectedAruco']

        # cv.aruco.getPredefinedDictionary
        arucoDictID = eval('cv.aruco.' + self.ARUCO_DICT)
        self.arucoDict = cv.aruco.getPredefinedDictionary(arucoDictID)

        self.board = cv.aruco.CharucoBoard((self.SQUARES_VERTICALLY, self.SQUARES_HORIZONTALLY), self.SQUARE_LENGTH,
                                           self.MARKER_LENGTH, self.arucoDict)

    # 获取图片中所有 ArUco 码的位置
    def _getAllArucoPosition(self, image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        parameters = cv.aruco.DetectorParameters()
        aruco_detector = cv.aruco.ArucoDetector(self.arucoDict)
        marker_corners, marker_ids, rejected_candidates = aruco_detector.detectMarkers(gray)
        if marker_ids is None:
            raise Exception("No ArUco markers detected.")
        marker_corners = np.array(marker_corners)
        marker_ids = np.array(marker_ids)

        image_copy = image.copy()
        cv.aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
        cv.imwrite('data/debug.png', image_copy)

        assert marker_corners.shape[1] == 1
        assert marker_corners.shape[2] == 4
        assert marker_corners.shape[3] == 2
        assert marker_ids.shape[1] == 1
        assert marker_ids.shape[0] == marker_corners.shape[0]

        # reshape to (n, 4, 2) and (n)
        marker_corners = marker_corners.reshape(-1, 4, 2)
        marker_ids = marker_ids.reshape(-1)
        return marker_corners, marker_ids

    # 过滤出指定 ArUco 码的位置（oneloc 任务专用）
    def _filterArucoPosition(self, marker_corners, marker_ids, selectedAruco):
        selectedMarkerCorners = []
        selectedMarkerIds = []
        for arucoID, marker_corner in zip(marker_ids, marker_corners):
            if arucoID in selectedAruco:
                selectedMarkerCorners.append(marker_corner)
                selectedMarkerIds.append(arucoID)
        selectedMarkerCorners = np.array(selectedMarkerCorners)
        selectedMarkerIds = np.array(selectedMarkerIds)
        return selectedMarkerCorners, selectedMarkerIds

    # 输入 (n, 4, 2) 的角点位置，返回 flatten 后的结果
    def _getImagePoints(self, marker_corners):
        image_points = np.vstack(marker_corners)
        ic(image_points.shape)
        return image_points

    # 获得角点在世界坐标系中的位置，输入所有 ArUco 码的四个角点和 ID，返回 flatten 后的结果
    def _getObjectPoints(self, selectedMarkerIds):
        squareCount = self.SQUARES_VERTICALLY * self.SQUARES_HORIZONTALLY // 2
        aruco_cnt = 0
        aruco_object_points = np.empty((squareCount, 4, 3), dtype=np.float32)

        halfMarkerLength = self.MARKER_LENGTH / 2
        center_object_point = np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]]) * halfMarkerLength

        for i in range(0, self.SQUARES_HORIZONTALLY):
            for j in range(0, self.SQUARES_VERTICALLY):
                if (i + j) % 2 == 0:
                    continue

                aruco_object_points[aruco_cnt] = center_object_point
                aruco_object_points[aruco_cnt, :, 1] += float(i) * self.SQUARE_LENGTH
                aruco_object_points[aruco_cnt, :, 0] += float(j - 1) * self.SQUARE_LENGTH
                aruco_cnt += 1

        object_points = np.array([aruco_object_points[markerID] for markerID in selectedMarkerIds.flatten()])

        assert object_points.shape[1] == 4
        assert object_points.shape[2] == 3
        object_points = object_points.reshape(-1, 3)

        # resultObjectPoints = center_object_point
        resultObjectPoints = object_points
        return resultObjectPoints

    def multiloc(self):
        allImageFile = getAllImage(self.imageAndPoseDirectory)
        imageLength = len(allImageFile)
        np.random.shuffle(allImageFile)
        all_cTt = []

        # 从每一张图片中获取 cTt
        for image_file in allImageFile:
            image = cv.imread(image_file)
            ic(image_file)

            # 所有 ArUco 码的角点位置
            marker_corners, marker_ids = self._getAllArucoPosition(image)

            # 过滤出指定 ArUco 码角点位置
            selectedAruco = self.selectedAruco
            selectedMarkerCorners, selectedMarkerIds = self._filterArucoPosition(marker_corners, marker_ids,
                                                                                 selectedAruco)
            # ic(selectedMarkerIds)
            # ic(selectedMarkerCorners)

            # image_copy = image.copy()
            # cv.aruco.drawDetectedMarkers(image_copy, selectedMarkerCorners, selectedMarkerIds)
            # cv.imshow('aruco_pose', image_copy)
            # cv.waitKey(0)

            # 获得 target 坐标系下的 aruco 码角点位置
            object_points = self._getObjectPoints(selectedMarkerIds)
            image_points = self._getImagePoints(selectedMarkerCorners)
            # ic(object_points)
            newObjectPoints, rectanglePoints, center = movePointsToRectangleCenter(object_points)
            ic(newObjectPoints)

            # cornerSubPix
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            winSize = (5, 5)
            zeroZone = (-1, -1)
            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 40, 0.001)
            image_points = cv.cornerSubPix(gray, image_points, winSize, zeroZone, criteria)

            # 计算 target 坐标系下的相机位姿
            cTt = self.calculate_cTt(newObjectPoints, image_points, camera_matrix, dist_coeffs)
            ic(cTt)

            all_cTt.append(cTt)

        return self.calculateLoc(all_cTt, imageLength, allImageFile)

    def calib(self):
        allImageFile = getAllImage(self.imageAndPoseDirectory)
        np.random.shuffle(allImageFile)
        imageLength = len(allImageFile)

        all_charuco_corners = []
        all_charuco_ids = []

        for image_file in allImageFile:
            image = cv.imread(image_file)
            ic(image_file)

            charucoDetector = cv.aruco.CharucoDetector(self.board)
            charuco_corners, charuco_ids, marker_corners, marker_ids = charucoDetector.detectBoard(image)

            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)

        # 相机标定
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv.aruco.calibrateCameraCharuco(
            all_charuco_corners, all_charuco_ids, self.board, image.shape[:2][::-1], None, None)
        ic(camera_matrix)
        ic(dist_coeffs)

        # write to matrix/camera_matrix.txt

        np.savetxt(f"matrix/{self.config['arm']}_camera_matrix.txt", camera_matrix, fmt='%.6f', delimiter=',')
        np.savetxt(f"matrix/{self.config['arm']}_dist_coeffs.txt", dist_coeffs, fmt='%.6f', delimiter=',')
        gTc = self.calculateCalib(imageLength, allImageFile, rvecs, tvecs)
        np.savetxt(f"matrix/{self.config['arm']}_gTc.txt", gTc, fmt='%.6f', delimiter=',')
        return gTc

# Aruco二维码
class Aruco(Pattern):
    def __init__(self, config):
        super(Aruco, self).__init__(config)
        assert self.type == 'aruco'
        # assert self.task != 'calib'

        # 字典类型配置（保留原注释）
        # DICT_ARUCO_ORIGINAL：原始 ArUco 字典，包含 1024 个标记，5x5 像素网格，适用于通用场景
        # DICT_5X5_250：5x5 像素网格，250 个标记，适合标记数量需求低的场景
        self.ARUCO_DICT = config['aruco']['ARUCO_DICT']
        # 解析目标ID-尺寸字典（键：ID字符串 → 转为整数，值：尺寸，此处仅用键筛选ID）
        self.MARKER_ID_AND_LENGTH = config['aruco']['MARKER_ID_AND_LENGTH']
        # 提取所有目标ID（转为整数，避免与检出的整数ID匹配偏差）
        self.target_marker_ids = [int(id_str) for id_str in self.MARKER_ID_AND_LENGTH.keys()]
        self.marker_lengths = {}
        self.object_points = {}
        for marker_id in self.target_marker_ids:
            self.marker_lengths[marker_id] = self.MARKER_ID_AND_LENGTH[str(marker_id)]  # 注意转为字符串键
            half_len = self.marker_lengths[marker_id] / 2
            # 计算位姿（使用当前ID的角点和尺寸）
            self.object_points[marker_id] = generateRectangle(-half_len, half_len, -half_len, half_len)

        arucoDictID = eval('cv.aruco.' + self.ARUCO_DICT)
        self.arucoDict = cv.aruco.getPredefinedDictionary(arucoDictID)

    # 获取图片中所有 ArUco 码的位置
    def _getArucoPosition(self, image, debug_flag=True):
        # 1. 图像格式转换（RealSense帧 → numpy数组）
        if isinstance(image, pyrealsense2.video_frame):
            image = np.asarray(image.get_data())  # 转为BGR格式，兼容OpenCV

        # 2. 检测ArUco二维码
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        parameters = cv.aruco.DetectorParameters()
        aruco_detector = cv.aruco.ArucoDetector(self.arucoDict)
        marker_corners, marker_ids, rejected_candidates = aruco_detector.detectMarkers(gray)

        # 3. 处理检出结果：构建“目标ID→角点”的映射（未检出为None）
        # 初始化结果字典：所有目标ID默认设为None
        target_marker_dict = {target_id: None for target_id in self.target_marker_ids}

        # 若有检出二维码，筛选目标ID并赋值角点
        if marker_ids is not None and len(marker_ids) > 0:
            # 打印原始检出信息
            detected_ids = marker_ids.flatten()  # 展平ID数组（避免维度问题）
            print(f"原始检出 {len(detected_ids)} 个ArUco二维码，IDs：{detected_ids}")

            # 遍历检出的二维码，匹配目标ID
            for corners, detected_id in zip(marker_corners, detected_ids):
                # 若当前检出ID在目标列表中，记录角点（corners[0]：提取角点坐标数组，形状(4,2)）
                if detected_id in self.target_marker_ids:
                    target_marker_dict[detected_id] = np.asarray(corners[0], dtype=np.float32)
                    # print(f"目标二维码匹配成功：ID={detected_id}，角点已保留")
                else:
                    # print(f"非目标二维码：ID={detected_id}，已过滤")
                    pass
        else:
            rospy.logerr("视野中未检测到任何ArUco二维码")

        # 4. 调试模式：绘制“仅目标二维码”（可选，更清晰区分目标）
        if debug_flag:
            image_copy = image.copy()
            # 收集需绘制的目标二维码（ID+角点）
            target_corners_to_draw = []
            target_ids_to_draw = []
            for target_id, corners in target_marker_dict.items():
                if corners is not None:
                    corner_group = np.expand_dims(corners, axis=0)
                    target_corners_to_draw.append(corner_group)
                    target_ids_to_draw.append(target_id)
            # 绘制目标二维码（绿色框+ID标注）
            if target_corners_to_draw:
                cv.aruco.drawDetectedMarkers(
                    image_copy, 
                    target_corners_to_draw, 
                    np.array(target_ids_to_draw)  # 转为数组，兼容API
                )
                # 标注未检出的目标ID（红色文字，提示缺失）
                for target_id, corners in target_marker_dict.items():
                    if corners is None:
                        cv.putText(
                            image_copy,
                            f"目标ID={target_id}（未检出）",
                            (50, 50 + list(target_marker_dict.keys()).index(target_id) * 30),  # 分行显示
                            cv.FONT_HERSHEY_SIMPLEX,
                            3, (0, 0, 255), 5  # 红色文字，粗2px
                        )
            debug_image_path = os.path.join(self.imageAndPoseDirectory, 'debug_target_aruco.png')
            cv.imwrite(debug_image_path, image_copy)
            rospy.loginfo(f"调试图已保存至 {debug_image_path}（仅含目标二维码）")

        # 5. 返回结果：目标ID→角点的字典（未检出为None）
        # print(f"\n最终目标二维码结果字典：{target_marker_dict}")
        return target_marker_dict
    
    def realtime(self, image, pose):
        # 获取目标二维码字典
        stime=time.time()
        print(f'识别时间 {(time.time()-stime)*1000:.1f} ms')
        target_marker_dict = self._getArucoPosition(image, debug_flag=True)
        target_marker_bTts = {}
        # 遍历目标ID，计算每个检出二维码的位姿
        for marker_id, corners in target_marker_dict.items():
            if corners is None:
                rospy.logwarn(f"ID={marker_id} 未检出，跳过位姿计算")
                continue
            
            # 计算位姿（使用当前ID的角点和尺寸）
            bTt = self.calculateOneloc(self.object_points[marker_id], corners, trans_pose_to_bTg(pose))
            self.publish_tf(bTt, 'b', 't', f'aruco_{marker_id}_marker')
            target_marker_bTts[marker_id] = bTt
            # print(f"ID={marker_id} 的位姿矩阵：\n{bTt}")
        
        print(f'单次定位整体时间:{(time.time()-stime)*1000:.1f} ms 帧率:{1/(time.time()-stime):.0f}帧/s')
    
        return target_marker_bTts

    def multiloc(self):
        # 打印开始定位的信息
        print('~~~~~~~~开始多帧定位~~~~~~~~')
        image_and_bTg = read_all_image_and_bTg(self.imageAndPoseDirectory)
        # print('all_bTg=',all_bTg)
        all_cTt = []
        all_bTg = []
        marker_names = []
        target_marker_id=-1
        # print('all_bTg counts=',len(all_bTg_origin))
        # print('allImageFile:',allImageFile)
        # 从每一张图片中获取 cTt，仅保留识别成功的图片
        for image_file,bTg in image_and_bTg:
            # 读取图像
            image = cv.imread(image_file)
            # ic(image_file)
            # 获取图像中所有 ArUco 码的角点位置和 ID
            target_marker_dict = self._getArucoPosition(image)
            if len(target_marker_dict)!=1:
                rospy.logwarn(f'目标Aruco二维码不唯一，移除图片 {image_file}')
                continue
            marker_id=next(iter(target_marker_dict.keys()))
            corners=next(iter(target_marker_dict.values()))
            if corners is None:
                rospy.logwarn(f'未识别到Aruco({marker_id})二维码，移除图片 {image_file}')
                continue
            # 计算目标物体相对于相机的位姿 cTt
            cTt = self.calculate_cTt(self.object_points[marker_id], corners, self.camera_matrix, self.dist_coeffs)
            # ic(cTt)
            all_cTt.append(cTt)
            all_bTg.append(bTg)
            marker_names.append(f'aruco_{marker_id}_marker({len(all_cTt)-1})')
            target_marker_id=marker_id
            # print(bTg)
        imageLength=len(all_cTt)
        if imageLength==0:
            rospy.logerr(f'无有效照片，无法进行多帧定位')
            return None
        if imageLength<4:
            rospy.logerr(f'识别成功图片数量{imageLength}<4，无法进行多帧定位')
            return None
        # 打印完成定位的信息
        mean_bTt=self.calculateLoc(imageLength, all_cTt, all_bTg, marker_names)
        self.publish_tf(mean_bTt, 'b', 't', f'aruco_{target_marker_id}marker_mean')
        print('~~~~~~~~结束多帧定位~~~~~~~~')
        # 调用 calculateLoc 方法计算最终的定位结果
        return mean_bTt
# 圆点标定板
class CircleGrid(Pattern):
    def __init__(self, config):
        super(CircleGrid, self).__init__(config)
        assert self.type == 'circleGrid'

        # 圆心间距
        self.circleDistance = config['circleGrid']['circleDistance']
        # 圆形半径
        self.circleRadius = config['circleGrid']['circleRadius']
        # 行数/列数
        self.circlePerRow = config['circleGrid']['circlePerRow']
        self.circleGridObjectPositions=self.getObjectPoints()
    def _detectCircleGrid(self, image_file=None, image_=None):
        if image_file is not None:
            ic(image_file)
            image = cv.imread(image_file)
        elif image_ is not None:
            if isinstance(image_, pyrealsense2.video_frame):
                # 转换为NumPy数组（BGR格式，与cv.imread保持一致）
                image = np.asarray(image_.get_data())
            else:
                # 若已是numpy数组则直接使用
                image = image_
        else:
            rospy.logerr(f'未传入图像，无法识别')
            return None
        # 所有 ArUco 码的角点位置
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        patternSize = (self.circlePerRow, self.circlePerRow)
        #cv.imwrite('data/gray.png', gray.copy())

        params = cv.SimpleBlobDetector.Params()
        ic(params.maxArea)
        ic(params.blobColor)
        detector = cv.SimpleBlobDetector.create(params)

        # keypoints = detector.detect(gray)
        # ic(len(keypoints))

        # ic(params.minThreshold)
        retval, centers = cv.findCirclesGrid(gray,
                                             patternSize,
                                             flags=cv.CALIB_CB_SYMMETRIC_GRID | cv.CALIB_CB_CLUSTERING,
                                             blobDetector=detector)
        if centers is None:
            rospy.logerr(f"视野中未检测到圆点标定板，识别失败")
            return None
        assert centers.shape == (self.circlePerRow * self.circlePerRow, 1, 2)
        centersGrid = centers.reshape(self.circlePerRow, self.circlePerRow, 2)

        allCenterGrid = []
        allCornerCenters = []

        for i in range(4):
            cornerCenter = centersGrid[0][0]
            allCornerCenters.append(cornerCenter)
            allCenterGrid.append(centersGrid)
            centersGrid = np.rot90(centersGrid)

        binarized = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)

        def getGray(image, centerGrid):
            center = np.mean(centerGrid.reshape(self.circlePerRow * self.circlePerRow, 2), axis=0)
            fourCorner = [centerGrid[0][0], centerGrid[-1][0], centerGrid[-1][-1], centerGrid[0][-1]]
            fourCornerExtend = [center + 5 / 4 * (corner - center) for corner in fourCorner]
            # 仿射变换到正方形
            squareSize = 300
            destPoints = np.array([[0, 0], [0, squareSize], [squareSize, squareSize], [squareSize, 0]], np.float32)
            M = cv.getPerspectiveTransform(np.array(fourCornerExtend, np.float32), destPoints)
            image = cv.warpPerspective(image, M, (squareSize, squareSize))
            # cv.imshow('image', image)
            #cv.waitKey(0)

            # crop
            cornerSquareSize = squareSize // 16
            meanGray = np.mean(image[:cornerSquareSize, :cornerSquareSize])
            return meanGray

        allGray = [getGray(image, centerGrid) for centerGrid in allCenterGrid]

        # 选择最小灰度的 index 的 centerGrid
        minIndex = np.argmin(allGray)
        ic(minIndex)
        ic(allGray)
        allGray.sort()
        if allGray[1] - allGray[0] <= 8:
            rospy.logerr(f"图像多个候选区域亮度接近，可能存在检测误差，识别失败 delta={allGray[1] - allGray[0]}")
            return None
        centersGrid = allCenterGrid[minIndex]
        centers = centersGrid.reshape(self.circlePerRow * self.circlePerRow, 1, 2)

        # image
        #image_copy = image.copy()
        #cv.drawChessboardCorners(image_copy, patternSize, centers, retval)
        #cv.imwrite('data/debug/' + image_file.split('/')[-1], image_copy)

        # ic(retval)
        # ic(len(centers))
        assert len(centers) == self.circlePerRow * self.circlePerRow
        centers = centers.reshape(self.circlePerRow * self.circlePerRow, 2)
        return centers

    def getObjectPoints(self):
        circleGridObjectPositions = np.zeros((self.circlePerRow * self.circlePerRow, 3), np.float32)
        for i in range(self.circlePerRow):
            for j in range(self.circlePerRow):
                circleGridObjectPositions[i * self.circlePerRow +
                                          j] = [j * self.circleDistance, i * self.circleDistance, 0]

        # move to center
        circleGridObjectPositions = circleGridObjectPositions - np.mean(circleGridObjectPositions, axis=0)
        return circleGridObjectPositions

    def calib(self):
        image_and_bTt = read_all_image_and_bTg(self.imageAndPoseDirectory)
        all_bTg = []
        # print('all_bTg=',all_bTg)
        all_center = []
        for image_file,bTt in image_and_bTt:
            print(f'识别{image_file}')
            centers = self._detectCircleGrid(image_file)
            if centers is None:
                print(f"识别失败，从列表中移除图片: {image_file}")
                continue  # 识别失败，不加入有效列表
            
            # 识别成功，同时保存有效图片路径和中心点
            # all_bTg.append(all_bTg_origin[len(valid_image_files)])
            all_bTg.append(bTt)
            all_center.append(centers)
        imageLength=len(all_bTg)
        if imageLength==0:
            rospy.logerr(f'无有效照片，无法进行手眼标定')
            return None
        if imageLength<4:
            rospy.logerr(f'识别成功图片数量{imageLength}<4，无法进行手眼标定')
            return None
        allObjectPositions = [self.circleGridObjectPositions for _ in range(imageLength)]
        print(f'imageLength={imageLength}')

        image1 = cv.imread(image_and_bTt[0][0])
        gray = cv.cvtColor(image1, cv.COLOR_BGR2GRAY)
        ic(gray.shape[::-1])
        ret, camera_matrix, dist_coeffs, all_rvecs, all_tvecs = cv.calibrateCamera(allObjectPositions,
                                                                                   all_center,
                                                                                   gray.shape[::-1],
                                                                                   None,
                                                                                   None,
                                                                                   flags=cv.CALIB_FIX_K3
                                                                                   | cv.CALIB_FIX_K2)

        # showUndistortedImage(allImageFile[0], camera_matrix, dist_coeffs)
        ic(camera_matrix, dist_coeffs)
        all_rvecs = np.array(all_rvecs)

        # all_bTg = read_all_bTg(self.imageAndPoseDirectory, allImageFile)
        np.savetxt(f"matrix/{self.config['arm']}_camera_matrix.txt", camera_matrix, fmt='%.6f', delimiter=',')
        np.savetxt(f"matrix/{self.config['arm']}_dist_coeffs.txt", dist_coeffs, fmt='%.6f', delimiter=',')
        ic(camera_matrix)
        ic(dist_coeffs)
        gTc = self.calculateCalib(imageLength, all_bTg, all_rvecs, all_tvecs)
        self.publish_tf(gTc, 'g', 'c')
        np.savetxt(f"matrix/{self.config['arm']}_gTc.txt", gTc, fmt='%.6f', delimiter=',')
        return gTc

    def multiloc(self):
        print('~~~~~~~~开始多帧定位~~~~~~~~')
        image_and_bTg = read_all_image_and_bTg(self.imageAndPoseDirectory)
        # print('all_bTg=',all_bTg)
        all_center = []
        all_cTt = []
        all_bTg = []
        marker_names = []

        # 从每一张图片中获取 cTt，仅保留识别成功的图片
        for image_file,bTg in image_and_bTg:
            print(f'识别{image_file}')
            centers = self._detectCircleGrid(image_file)
            if centers is None:
                print(f"识别失败，从列表中移除图片: {image_file}")
                continue  # 识别失败，不加入有效列表
            
            # 识别成功，同时保存有效图片路径和中心点
            # all_bTg.append(all_bTg_origin[len(valid_image_files)])
            all_bTg.append(bTg)
            all_center.append(centers)
            marker_names.append(f'circleGrid_marker({len(all_center)-1})')
        imageLength=len(all_bTg)
        if imageLength==0:
            rospy.logerr(f'无有效照片，无法进行多帧定位')
            return None
        if imageLength<4:
            rospy.logerr(f'识别成功图片数量{imageLength}<4，无法进行多帧定位')
            return None

        # 计算 target 坐标系下的相机位姿
        for i,centers in enumerate(all_center):
            # ic(centers.shape)
            # reshape to (n, 2)
            camera_matrix = self.camera_matrix
            dist_coeffs = self.dist_coeffs
            cTt = self.calculate_cTt(self.circleGridObjectPositions, centers, camera_matrix, dist_coeffs)
            # ic(cTt)
            all_cTt.append(cTt)
        # np.savetxt(all_cTt_file, np.array(all_cTt).flatten(), fmt='%.6f', delimiter=',')

        imageLength=len(all_cTt)
        mean_bTt=self.calculateLoc(imageLength, all_cTt, all_bTg, marker_names)
        self.publish_tf(mean_bTt, 'b', 't', f'circleGrid_marker_mean')
        print('~~~~~~~~结束多帧定位~~~~~~~~')
        return mean_bTt
    
    def realtime(self,image,pose):
        # 所有 circleGrid 的点位置
        stime=time.time()
        centers = self._detectCircleGrid(image_=image)
        print(f'识别时间 {(time.time()-stime)*1000:.1f} ms')
        if centers is None:
            return None
        # 获得 target 坐标系下的 circleGrid 的点位置
        newObjectPoints, rectanglePoints, center = movePointsToRectangleCenter(self.circleGridObjectPositions)

        image_points = centers

        # 计算 target 坐标系下的相机位姿
        bTt = self.calculateOneloc(newObjectPoints, image_points, trans_pose_to_bTg(pose))
        self.publish_tf(bTt, 'b', 't', 'circleGrid')
        print(f'单次定位整体时间:{(time.time()-stime)*1000:.1f} ms 帧率:{1/(time.time()-stime):.0f}帧/s')
        # print('base to target 的齐次矩阵是：')
        # print(bTt)
        return bTt


if __name__ == '__main__':
    # 读取参数 config.json
    assert len(sys.argv) == 2
    file = sys.argv[1]
    config = readConfig(file)

    type = config['type']
    task = config['task']

    if type == 'charuco':
        pattern = Charuco(config)
    elif type == 'aruco':
        pattern = Aruco(config)
    elif type == 'circleGrid':
        pattern = CircleGrid(config)

    if task == 'oneloc':
        result = pattern.oneloc()
    elif task == 'calib':
        result = pattern.calib()
    elif task == 'multiloc':
        result = pattern.multiloc()
