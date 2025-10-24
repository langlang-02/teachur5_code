#!/bin/bash

# 启动 roscore
gnome-terminal --tab -- bash -c  "roscore; exec bash"
sleep 2

# 启动UR机器人驱动
gnome-terminal --tab -- bash -c "roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.60.49; exec bash"
sleep 5

# 启动双机器人MoveIt配置
gnome-terminal --tab -- bash -c "roslaunch single_robot_moveit_config ur5e_moveit_planning_execution.launch; exec bash"

# # 启动RViz
gnome-terminal --tab -- bash -c "roslaunch manipulator single_arm_rviz.launch; exec bash"
# sleep 5


# # 启动夹爪服务
gnome-terminal --tab -- bash -c "roslaunch gripper_modbus Gripper_ModbusControl.launch type:=RGI port:=/dev/ttyUSB0 prefix:=RGI; exec bash"
gnome-terminal --tab -- bash -c "roslaunch gripper_modbus Gripper_ModbusControl.launch type:=PGI port:=/dev/ttyUSB1 prefix:=PGI; exec bash"

#sudo udevadm trigger
#ll /dev/ |grep ttyUSB

# 启动A_gel_start
gnome-terminal --tab -- bash -c "roslaunch manipulator A_robot_start.launch Dual_arm_flag:=false Dashboard_flag:=true Locator_flag:=false Load_JPfile:=robot; exec bash"
sleep 10

gnome-terminal --tab -- bash -c "roslaunch manipulator B_JP_record.launch Record_JPfile:=robot; exec bash"

roslaunch js_control js_control.launch switch_mode:=1 control_mode:=0
