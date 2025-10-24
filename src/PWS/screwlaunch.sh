#!/bin/zsh

# 启动 roscore
gnome-terminal --tab -- zsh -c  "roscore; exec zsh"
sleep 2

# 启动UR机器人驱动
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch ur_robot_driver dual_ur_bringup.launch body_arm_flag:=true; exec zsh"
sleep 5

# 启动双机器人MoveIt配置
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch dual_moveit_config dual_moveit_planning_execution.launch; exec zsh"

# # 启动RViz
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch manipulator dual_arm_rviz.launch; exec zsh"
# sleep 5

# # 启动移液枪
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; rosrun adp1000 adp1000.py; exec zsh"
# sleep 5

# # 启动夹爪服务
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch gripper_modbus Gripper_ModbusControl.launch ; exec zsh"
# 启动天平通信
# gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch balance_com Balance_com.launch ; exec zsh"

#sudo udevadm trigger
#ll /dev/ |grep ttyUSB

# 发布末端执行器tf
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; rosrun manipulator tool_end_tf_publisher.py ; exec zsh"
sleep 5

# 启动A_gel_start
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch manipulator A_robot_start.launch Dashboard_flag:=true ; exec zsh"
sleep 20

source ~/B_workspace/screw_dual_robot/devel/setup.zsh
roslaunch js_control js_control.launch switch_mode:=7 control_mode:=0
