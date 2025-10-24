# !/bin/bash

# 启动手柄
# gnome-terminal --tab -- bash -c "source ~/B_workspace/screw_dual_robot/devel/setup.bash; roslaunch js_control js_control.launch; exec bash"

# 启动标点程序
gnome-terminal --tab -- bash -c "source ~/B_workspace/screw_dual_robot/devel/setup.bash; roslaunch manipulator B_JP_record.launch; exec bash"

# 启动标点程序
#gnome-terminal --tab -- bash -c "source ~/B_workspace/screw_dual_robot/devel/setup.bash; rosrun pgi140 pgitest; exec bash"
gnome-terminal --tab -- bash -c "source ~/B_workspace/screw_dual_robot/devel/setup.bash; rosrun gripper_modbus pgitest; exec bash"
#启动plot
# gnome-terminal --tab -- bash -c "force; exec bash"


