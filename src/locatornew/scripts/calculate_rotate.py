#!/usr/bin/env python3
# 导入必要的第三方库
import cv2 as cv          # OpenCV库，常用于图像处理（本脚本未直接使用，但可能为后续扩展预留）
import numpy as np        # NumPy库，用于数值计算、矩阵操作（核心库）
from icecream import ic   # 增强版打印工具，用于调试时输出变量信息（带文件名、行号）
from scipy.spatial.transform import Rotation as R  # SciPy库的旋转模块，用于旋转矩阵与旋转向量转换
import os                 # OS库，用于文件路径操作、目录切换
import json               # JSON库，用于JSON格式数据读写（本脚本未直接使用，可能为配置读取预留）
import sys                # Sys库，用于系统相关操作（本脚本未直接使用，可能为参数解析预留）


# -------------------------- 路径初始化：确保脚本基于自身所在目录运行 --------------------------
# 获取当前脚本的绝对路径（如 /home/user/project/transform.py）
currentFilePath = os.path.abspath(__file__)
# 提取当前脚本所在的目录路径（如 /home/user/project）
currentDir = os.path.dirname(currentFilePath)
# 将工作目录切换到脚本所在目录，避免路径依赖问题（确保后续读取文件时路径正确）
os.chdir(currentDir)

# 设置NumPy打印格式：保留3位小数，抑制科学计数法（便于调试查看矩阵数值）
np.set_printoptions(precision=3, suppress=True)


def rotateMatrix(targetXYZ_tTc):
    """
    生成目标方向对应的4个候选旋转矩阵（用于姿态解算的多解场景）
    
    功能：根据目标点在二维码坐标系下的3D坐标，计算出满足方向约束的4个旋转矩阵
    适用场景：机械臂抓取等需要确定目标姿态的任务，可能存在多个合法姿态解
    
    Args:
        targetXYZ_tTc (np.ndarray): 目标点在二维码坐标系（t）下的3D坐标，形状(3,)
                                    格式：[x, y, z]（单位需与其他参数统一，如毫米）
    
    Returns:
        tuple: 4个候选旋转矩阵 A1~A4，每个矩阵形状(3,3)，均为标准正交旋转矩阵
    """
    # 1. 对目标点坐标进行归一化（转为单位向量，消除距离影响，仅保留方向信息）
    # np.linalg.norm 计算向量的L2范数（即模长）
    normalized = targetXYZ_tTc / np.linalg.norm(targetXYZ_tTc)
    # 提取归一化后的x、y分量（用于后续旋转矩阵参数计算）
    a = normalized[0]
    b = normalized[1]
    # 调试输出：打印归一化后的x、y分量（确认方向计算基础）
    ic(a, b)

    # 2. 计算旋转矩阵的基础参数（基于预设的数学推导公式，对应4种姿态解）
    # 公式来源：根据目标方向的单位向量，推导满足约束的旋转矩阵元素
    # 注：分母 np.sqrt(1 - b²) 需确保 b² < 1（即目标点不沿y轴正方向/负方向，否则需特殊处理）
    p = -((a * b) / np.sqrt(1 - b**2))          # 旋转矩阵参数p
    q = np.sqrt(1 - b**2)                       # 旋转矩阵参数q
    r = a / np.sqrt(1 - b**2)                   # 旋转矩阵参数r
    # 计算参数s和k：基于x、y分量推导，体现z方向的约束
    s = -b * np.sqrt((-1 + a**2 + b**2) / (-1 + b**2))
    k = -np.sqrt((-1 + a**2 + b**2) / (-1 + b**2))

    # 3. 构建4个候选旋转矩阵（每个矩阵由3个列向量组成，列向量需满足标准正交性）
    # 旋转矩阵的列向量：分别对应目标坐标系的x、y、z轴在参考系下的方向
    # A1~A4 对应4种不同的姿态解（如抓取时的正/反、左/右朝向）
    v1 = np.array([k, 0, r])    # 第1列向量（x轴方向）
    v2 = np.array([p, q, s])    # 第2列向量（y轴方向）
    # 第3列向量（z轴方向）：由单位向量的z分量推导（1 - a² - b² 是归一化后z分量的平方）
    v3 = np.array([-a, -b, -np.sqrt(1 - a**2 - b**2)])
    
    # 构建并转置矩阵（确保列向量组成旋转矩阵，而非行向量）
    A1 = np.array([v1, v2, v3]).T  # 候选旋转矩阵1
    A2 = np.array([v1, -v2, v3]).T # 候选旋转矩阵2（y轴反向）
    A3 = np.array([-v1, v2, v3]).T # 候选旋转矩阵3（x轴反向）
    A4 = np.array([-v1, -v2, v3]).T# 候选旋转矩阵4（x、y轴均反向）
    
    return A1, A2, A3, A4


def transform_xyz_tTc_to_bTg(targetXYZ_tTc, bTt, gTc, matrix, camera_matrix, dist_coeffs):
    """
    坐标变换核心函数：将目标点从二维码坐标系转换到机械臂基座坐标系，并输出位姿
    
    功能流程：
    1. 计算二维码到机械臂基座的逆变换（tTb）
    2. 生成目标点的候选旋转矩阵
    3. 构建目标点在二维码坐标系下的齐次变换矩阵
    4. 串联多坐标系变换，得到目标点在机械臂基座坐标系下的位姿
    5. 提取平移向量和旋转向量（符合机械臂控制的位姿格式）
    
    Args:
        targetXYZ_tTc (np.ndarray): 目标点在二维码坐标系（t）下的3D坐标，形状(3,)
                                    格式：[x, y, z]（单位需统一，如毫米）
        bTt (np.ndarray): 机械臂基座坐标系（b）到二维码坐标系（t）的齐次变换矩阵，形状(4,4)
                          格式：[R | t; 0 0 0 1]，R为3x3旋转矩阵，t为3x1平移向量
        matrix (str): 选择候选旋转矩阵的标识，可选值：'A1'/'A2'/'A3'/'A4'（对应不同姿态解）
    
    Returns:
        tuple: 
            translationVector (np.ndarray): 目标点在机械臂基座坐标系下的平移向量，形状(3,)
                                           格式：[x, y, z]（单位与输入一致）
            rotateVector (np.ndarray): 目标点在机械臂基座坐标系下的旋转向量（Rodrigues向量），形状(3,)
                                      格式：[rx, ry, rz]（单位：弧度，符合OpenCV/Robot标准）
    """
    # 调试输出：打印基座到二维码的变换矩阵（确认输入矩阵正确性）
    # ic(bTt)

    # 1. 计算二维码坐标系（t）到机械臂基座坐标系（b）的逆变换矩阵 tTb
    # 物理意义：将二维码坐标系下的点转换到基座坐标系（t = tTb * b）
    # np.linalg.inv 计算矩阵的逆（要求bTt为非奇异矩阵，即行列式≠0）
    tTb = np.linalg.inv(bTt)

    # 2. 调用rotateMatrix函数，生成目标点的4个候选旋转矩阵
    A1, A2, A3, A4 = rotateMatrix(targetXYZ_tTc)

    # 3. 根据输入参数matrix，选择对应的旋转矩阵（确定目标姿态解）
    # 注：注释部分为预留逻辑（根据目标点位置动态选择姿态解，如距离阈值判断）
    # if np.linalg.norm(t_bTc[:2]) > 500:  # 若目标点x-y平面距离超过500，选择A2
    #     A = A2
    # else:
    if matrix == 'A1':
        A = A1
    elif matrix == 'A2':
        A = A2
    elif matrix == 'A3':
        A = A3
    elif matrix == 'A4':
        A = A4

    # 4. 构建目标点在二维码坐标系（t）下的齐次变换矩阵 target_tTc
    # 齐次变换矩阵格式：[R | t; 0 0 0 1]，其中：
    # - R = A（目标点的旋转矩阵，即姿态）
    # - t = targetXYZ_tTc（目标点在二维码坐标系下的平移）
    target_tTc = np.eye(4)                # 初始化4x4单位矩阵
    target_tTc[:3, 3] = targetXYZ_tTc     # 上右3x1：平移向量（目标点坐标）
    target_tTc[:3, :3] = A                # 上左3x3：旋转矩阵（目标姿态）
    # 调试输出：打印目标点在二维码坐标系下的齐次变换矩阵
    # ic(target_tTc)

    # 5. 计算相机坐标系（c）到全局坐标系（g）的逆变换矩阵 cTg
    # 物理意义：将全局坐标系下的点转换到相机坐标系（g = cTg * c）
    # 由 gTc（全局到相机）求逆得到 cTg（相机到全局）
    cTg = np.linalg.inv(gTc)

    # 6. 串联多坐标系变换，得到目标点在机械臂基座坐标系（b）下的齐次变换矩阵 target_bTg
    # 变换逻辑：b → t → 目标 → c → g → b（需根据实际坐标系关系确认，此处为示例逻辑）
    # 公式推导：target_bTg = bTt（基座→二维码） @ target_tTc（二维码→目标） @ cTg（目标→相机→全局→基座）
    # 物理意义：通过该矩阵可将目标点从自身坐标系转换到基座坐标系
    target_bTg = bTt @ target_tTc @ cTg
    # 调试输出：打印目标点在基座坐标系下的齐次变换矩阵
    # ic(target_bTg)
    return target_bTg


# -------------------------- 主函数：脚本入口（用于测试坐标变换功能） --------------------------
if __name__ == '__main__':
    # 1. 加载测试数据：机械臂基座到二维码的变换矩阵（从文本文件读取）
    # oneloc_bTt.txt 为单帧定位结果，存储bTt矩阵（逗号分隔）
    bTt = np.loadtxt('matrix/oneloc_bTt.txt', delimiter=',')
    
    # 2. 定义测试目标点：在二维码坐标系下的3D坐标（示例值，需根据实际场景调整）
    targetXYZ_tTc = np.array([100, 100, 400])  # 单位：假设为毫米
    
    # 3. 调用坐标变换函数，选择A1作为姿态解（可切换为A2/A3/A4测试不同姿态）
    translationVector, rotateVector = transform_xyz_tTc_to_bTc(targetXYZ_tTc, bTt, 'A1')
    
    # 4. 调试输出：打印最终结果（目标点在基座坐标系下的平移和旋转）
    ic(translationVector)  # 输出平移向量：[x, y, z]
    ic(rotateVector)       # 输出旋转向量：[rx, ry, rz]（弧度）