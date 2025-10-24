#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2 as cv
from icecream import ic


# 旋转向量、平移向量 -> 矩阵
def rtToMatrix(rotvec, tvec):
    rotMatrix = cv.Rodrigues(rotvec)[0]
    mat = np.eye(4)
    mat[:3, :3] = rotMatrix
    mat[:3, 3] = tvec
    return mat


# 四元数 -> 旋转向量
def quaternionToRotvec(q):
    return R.from_quat(q).as_rotvec()


# 方向向量 -> 旋转
def vecToMatrix(vec):
    return R.from_rotvec(vec).as_matrix()


# 旋转矩阵 -> 四元数
def matrixToQuat(mat):
    rotation_mat = R.from_matrix(mat)
    return rotation_mat.as_quat()


# 旋转矩阵 -> 四元数
def quatToMatrix(quat):
    rot = R.from_quat(quat)  # 顺序为 (x, y, z, w)
    return rot.as_matrix()


# 旋转矩阵、平移向量 -> 矩阵
def rotateMatrixTranslationVectorToMatrix(rot, t):
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = t
    return mat


# 矩阵 -> 平移向量、四元数连接起来
def matrixToTQ(mat):
    rot = mat[:3, :3]
    t = mat[:3, 3]
    quaternion = matrixToQuat(rot)
    result = np.concatenate((t, quaternion))
    return result


# 平移向量、四元数 -> 矩阵
def tqToMatrix(tq):
    t = tq[:3]
    q = tq[3:]
    rot = quatToMatrix(q)
    return rotateMatrixTranslationVectorToMatrix(rot, t)
