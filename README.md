# 双臂协作机器人

## 启动流程

1. 启动机械臂镜像文件：

   left_robot:

   ```
   sudo docker run --rm -it -p 5900:5900 -p 6080:6080 --net ursim_net --name URSimleft --ip 192.168.131.11 ustcert/ursim_e
   ```

   right_robot:（开启后需将50002改为50006）

   ```
   sudo docker run --rm -it -p 3900:3900 -p 4080:4080 --net ursim_net --name URSimright --ip 192.168.131.12 ustcert/ursim_e
   ```

2. 启动**vncviewer**：

   ```
   vncviewer
   ```

3. 进入工作空间，以下命令在工作空间进行，之后开启4个终端)：

   ```
   source ./B_workspace/dual_robot/devel/setup.bash
   ```

   或者

   ```
   cd /home/klb/B_workspace/A_dual_robot/dual_robot
   source ./devel/setup.bash
   ```

4. 启动机械臂ROS驱动(需要指定**robot_ip**：

   ```
   roslaunch ur_robot_driver dual_ur_bringup.launch
   ```

   若驱动的为实体双臂，则需禁用镜像网卡(up是开启)

   ```
   sudo ifconfig br-b3838b5e49ca down
   ```

5. 若需使用dashboard命令开启实体目标机械臂

   left_robot:

   ```
   #开启并解除制动
   rosservice call /left_robot/ur_hardware_interface/dashboard/power_on
   rosservice call /left_robot/ur_hardware_interface/dashboard/brake_release 
   #若机械臂当前存在保护
   rosservice call /left_robot/ur_hardware_interface/dashboard/unlock_protective_stop
   #加载驱动文件
   rosservice call /left_robot/ur_hardware_interface/dashboard/load_program "filename: 'pro_1.urp'"
   rosservice call /left_robot/ur_hardware_interface/dashboard/play
   #终止当前驱动文件
   rosservice call /left_robot/ur_hardware_interface/dashboard/pause
   #关闭机械臂
   rosservice call /left_robot/ur_hardware_interface/dashboard/power_off
   ```

   right_robot:

   ```
   #开启并解除制动
   rosservice call /right_robot/ur_hardware_interface/dashboard/power_on
   rosservice call /right_robot/ur_hardware_interface/dashboard/brake_release
   #若机械臂当前存在保护
   rosservice call /right_robot/ur_hardware_interface/dashboard/unlock_protective_stop
   #加载驱动文件
   rosservice call /right_robot/ur_hardware_interface/dashboard/load_program "filename: 'pro_2.urp'" 
   rosservice call /right_robot/ur_hardware_interface/dashboard/play
   #终止当前驱动文件
   rosservice call /right_robot/ur_hardware_interface/dashboard/pause
   #关闭机械臂
   rosservice call /right_robot/ur_hardware_interface/dashboard/power_off
   ```

6. 启动**moveit！**

   ```
   roslaunch dual_moveit_config dual_moveit_planning_execution.launch
   ```

7. 启动**Rviz GUI**:

   ```
   roslaunch manipulator dual_arm_rviz.launch
   ```

8. 进行研磨操作（开启图像处理服务器）

   ```
   #开启图像处理服务器
   cd /home/klb/B_workspace/A_dual_robot/dual_robot
   source ./devel/setup.bash
   rosrun image_deal imagedeal_server.py
   #研磨操作
   rosrun manipulator A_manipulator
   ```



## 使用手柄控制

1. right_robot:（默认）

   ```
   roslaunch js_control js_control.launch goal_robot:="right_robot"
   ```


2. left_robot:

   ```
   roslaunch js_control js_control.launch goal_robot:="left_robot"
   ```

   

## PGI串口权限

1. 查看串口权限

   ```
   ls -l /dev|grep ttyUSB
   ```



2. 重新更新串口情况（重新加载.rules）

   ```
   sudo udevadm trigger
   ```



## 视觉定位

1. 启动realsensor2相机

   ```
   roslaunch realsense2_camera rs_camera.launch
   ```

2. 启用aruco_ros识别

   ```
   roslaunch aruco_ros single.launch markerId:=21 markerSize:=0.075 eye:="right"
   ```

3. 检查结果

   ```
   rostopic echo /aruco_single/pose
   rosrun image_view image_view image:=/aruco_single/result
   ```



## 光路转台控制

0. 预先准备

   0.1 将arduino通电，开开关；

   0.2 转台初始化，进行转动，直到一圈后停止。

1. 运行rosserial节点(/dev/ttyUSB0之后要做一个串口映射)

   ```
   rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
   ```

2. 运行电机角度通信节点

   ```
   source ./B_workspace/dual_robot/devel/setup.bash
   roslaunch manipulator B_topic_pubsub.launch target_topic:="turntable_angles"
   ```

3. 输入关节角变量

