# 重定位功能包（locatornew）使用说明

## 功能概述

`locatornew` 是一个基于ROS1的重定位功能包，用于实现双机械臂系统提供标定、定位等功能。该包支持多种标定板类型（圆形网格、Aruco标记、Charuco棋盘格），提供话题（Topic）和服务（Service）两种交互接口，可实现实时识别、单次定位、标定和多次定位等操作。

## 目录结构

```
locatornew/
├── config/                  # 配置文件目录
│   └── [站点名称].json      # 目标站点的定位参数配置文件
├── img_take/                # 图像与位姿数据存储目录
│   ├── [图像文件]           # 拍摄的标定板/标记图像
│   └── pose.txt             # 对应的机械臂位姿数据
├── matrix/                  # 矩阵变换矩阵存储目录
│   ├── *_camera_matrix.txt  # 相机内参矩阵（标定结果）
│   ├── *_dist_coeffs.txt    # 相机畸变系数（标定结果）
│   ├── *_gTc.txt            # 手眼标定变换矩阵（工具到相机）
│   └── oneloc_bTt.txt       # 单次定位变换矩阵（世界到目标）
├── scripts/                 # 核心代码目录
│   ├── location_service.py  # 定位服务主节点（提供Topic和Service接口）
│   ├── analyze.py           # 标定板识别模块
│   ├── capture.py           # 图像捕获与处理模块
│   └── robot_controller.py  # 机械臂控制接口（支持moveP和getPoseBase方法）
├── CMakeLists.txt           # 编译配置文件
├── package.xml              # ROS包信息文件
└── setup.py                 # Python模块配置文件
```

## 配置文件说明

`config` 目录下的JSON配置文件用于定义目标站点的定位参数，格式如下：

### 配置参数说明

| 一级参数 | 二级参数 | 说明 | 可选值/示例 |
|---------|---------|------|------------|
| station | - | 站点名称 | "uv" |
| arm | - | 机械臂名称 | "left" 或 "right" |
| task | - | 任务类型 | "realtime"（实时识别）、"oneloc"（单次定位）、"calib"（标定）、"multiloc"（多次定位） |
| type | - | 标定板类型 | "circleGrid"、"aruco"、"charuco" |
| matrix | - | 旋转矩阵选择 | "A1"、"A2"、"A3"、"A4" |
| imageAndPoseDirectory | - | 数据存储目录 | "imr_take/" |
| poseRelativeFile | - | 位姿数据文件名 | "pose.txt" |
| calibResultFile | - | 标定结果文件名 | "avg_tq_bTt.txt" |
| calibMethod | - | 标定算法 | "CALIB_HAND_EYE_TSAI"、"CALIB_HAND_EYE_PARK"等 |
| moveP_acc | - | 机械臂运动加速度 | 0.5（范围0-1） |
| moveP_vel | - | 机械臂运动速度 | 1.0（范围0-1） |
| calib | overwrite | 是否覆盖已有标定结果 | true 或 false |
| calib | minX/maxX | X轴标定范围（毫米） | -80/80 |
| calib | minY/maxY | Y轴标定范围（毫米） | -80/80 |
| calib | minZ/maxZ | Z轴标定范围（毫米） | 250/350 |
| calib | pointNumX/Y/Z | 各轴采样点数量 | 5/5/3（总点数≥4） |
| multiloc | minX/maxX等 | 定位范围参数 | 类似calib，范围可更小 |
| multiloc | allNumber | 定位采样点总数 | 20（≥4） |
| charuco | ARUCO_DICT | Aruco字典类型 | "DICT_5X5_250" |
| charuco | SQUARE_LENGTH | 棋盘格边长（毫米） | 40 |
| charuco | MARKER_LENGTH | 标记边长（毫米） | 30 |
| charuco | SQUARES_VERTICALLY/HORIZONTALLY | 棋盘格数量 | 14/9 |
| charuco | selectedAruco | 选中的标记ID列表 | [31] |
| aruco | ARUCO_DICT | Aruco字典类型 | "DICT_ARUCO_ORIGINAL" |
| aruco | MARKER_ID_AND_LENGTH | 标记ID与边长映射 | {"38": 75} |
| circleGrid | circleDistance | 圆心间距（毫米） | 7 |
| circleGrid | circleRadius | 圆半径（毫米） | 1.75 |
| circleGrid | circlePerRow | 每行圆数量 | 7 |

### 配置示例

```json
{
    "station": "uv",
    "arm": "left",
    "task": "calib",
    "type": "circleGrid",
    "matrix": "A1",
    "calibMethod": "CALIB_HAND_EYE_TSAI",
    "moveP_acc": 0.5,
    "moveP_vel": 1.0,
    "calib": {
        "overwrite": true,
        "minX": -80,
        "maxX": 80,
        "minY": -80,
        "maxY": 80,
        "minZ": 250,
        "maxZ": 350,
        "pointNumX": 5,
        "pointNumY": 5,
        "pointNumZ": 3
    },
    "circleGrid": {
        "circleDistance": 7,
        "circleRadius": 1.75,
        "circlePerRow": 7
    }
    // 其他参数省略...
}
```

## 核心功能模块

### 1. 定位服务节点（location_service.py）

该节点是功能包的核心，提供两种交互接口：

#### 话题接口（Topic）

- **订阅话题**：`/locator_topic`（消息类型：`std_msgs/String`）
  - 接收JSON格式的命令字符串，格式与配置文件中的顶级参数一致
  - 示例：`{"station": "uv", "arm": "left", "task": "oneloc"}`

- **发布话题**：
  - `/obj_to_robot_holdon`（`PoseStamped`）：目标到机器人的位姿变换
  - `/class_order_holdon`（`String`）：目标名称

#### 服务接口（Service）

- **服务名称**：`/locator_service`
- **服务类型**：自定义`Location`服务，定义如下：
  ```
  string command_json  # 输入：JSON格式的命令字符串
  ---
  string status        # 输出：处理状态信息
  bool result_valid    # 输出：结果是否有效
  float64[] cached_bTt # 输出：4x4变换矩阵（展平为16元素列表）
  string cached_station # 输出：缓存的站点名称
  ```

- **调用示例**：
  ```bash
  rosservice call /locator_service '{"command_json": "{\"station\": \"uv\", \"arm\": \"left\", \"task\": \"oneloc\"}"}'
  ```

### 2. 机械臂控制接口（robot_controller.py）

封装了机械臂的运动控制和位姿获取功能，主要方法：

- `moveP(speed, acce, pose, arm)`：控制机械臂以指定速度和加速度运动到指定位姿
  - `speed`：运动速度（0-1）
  - `acce`：运动加速度（0-1）
  - `pose`：目标位姿（7元素数组：[x, y, z, qx, qy, qz, qw]）
  - `arm`：机械臂标识（"L"或"R"）

- `getPoseBase(arm)`：获取机械臂在基坐标系下的位姿
  - 返回：7元素数组（[x, y, z, qx, qy, qz, qw]）

### 3. 标定结果处理

标定完成后，系统会自动保存以下结果：

- 相机内参和畸变系数：保存至`matrix`目录下的`*_camera_matrix.txt`和`*_dist_coeffs.txt`
- 手眼标定变换矩阵（工具到相机）：保存至`matrix`目录下的`*_gTc.txt`
- 若`calib.overwrite`设为`true`，标定结果会自动更新到机械臂的URDF模型（`arm_robot_description/urdf/dual_arm_robot.xacro`）

### 4. 重定位结果
重定位结束后会将重定位结果通过以下两个话题持续发布，方便其他节点实时获取定位信息：

- **目标到机器人的位姿变换话题**
- 话题名称：`/obj_to_robot_holdon`
- 消息类型：`geometry_msgs/PoseStamped`
- 发布频率：10Hz（每秒10次）
- 内容说明：
    - 位置信息（x, y, z）：取自变换矩阵`bTt`的第4列前3个元素（世界坐标系到目标的平移分量）
    - 姿态信息（四元数）：由变换矩阵`bTt`的3×3旋转部分转换而来
    - 参考坐标系：`world`（世界坐标系）

- **目标站点名称话题**
- 话题名称：`/class_order_holdon`
- 消息类型：`std_msgs/String`
- 发布频率：10Hz（与位姿话题同步）
- 内容说明：当前定位的目标站点名称（与配置文件中的`station`参数一致，如"uv"）

## 订阅示例

可通过以下命令查看实时发布的定位结果：

```bash
# 查看位姿变换数据
rostopic echo /obj_to_robot_holdon

# 查看目标站点名称
rostopic echo /class_order_holdon
```

这两个话题为系统中的其他模块（如机械臂控制、视觉显示）提供了便捷的定位结果获取方式，无需主动调用服务或话题请求。
## 使用步骤

### 1. 编译功能包

```bash
# 进入工作空间
cd /home/admin1/chem_dual_rob/
# 编译
catkin_make
# 刷新环境变量
source devel/setup.bash
```

### 2. 启动定位服务

```bash
roslaunch locatornew location_service.launch
```

### 3. 执行定位/标定任务

#### 通过服务调用（推荐）

```bash
# 单次定位示例
rosservice call /locator_service '{"command_json": "{\"station\": \"uv\", \"arm\": \"left\", \"task\": \"oneloc\"}"}'

# 标定示例
rosservice call /locator_service '{"command_json": "{\"station\": \"uv\", \"arm\": \"left\", \"task\": \"calib\", \"type\": \"circleGrid\"}"}'
```

#### 通过话题发布

```bash
# 发布单次定位命令
rostopic pub /locator_topic std_msgs/String "data: '{\"station\": \"uv\", \"arm\": \"left\", \"task\": \"oneloc\"}'"
```

## 注意事项

1. **配置文件**：使用前需在`config`目录下创建对应站点的配置文件，确保参数与实际硬件匹配
2. **机械臂接口**：若更换机械臂，只需修改`robot_controller.py`，确保实现`moveP`和`getPoseBase`方法
3. **标定板**：使用前需确保标定板参数（如尺寸、数量）与配置文件一致
4. **权限问题**：确保`scripts`目录下的Python文件具有可执行权限
5. **日志清理**：若ROS日志过大，可使用`rosclean purge`清理

## 故障排除

1. **模块导入错误**：确保所有依赖包已安装，工作空间已正确编译并刷新环境变量
2. **机械臂通信失败**：检查机械臂是否正常连接，`robot_controller.py`中的话题名称是否与机械臂驱动一致
3. **标定结果异常**：检查标定范围设置是否合理，确保标定板在整个标定过程中可见
4. **文件写入失败**：检查目录权限，确保程序有权限写入`img_take`和`matrix`目录

## 版本历史

- v1.0：初始版本，支持circleGrid、aruco和charuco标定板，提供Topic和Service接口