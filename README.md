<<<<<<< HEAD
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

=======
# teachur5_code



## Getting started

To make it easy for you to get started with GitLab, here's a list of recommended next steps.

Already a pro? Just edit this README.md and make it your own. Want to make it easy? [Use the template at the bottom](#editing-this-readme)!

## Add your files

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/topics/git/add_files/#add-files-to-a-git-repository) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://git.lug.ustc.edu.cn/Mathison/teachur5_code.git
git branch -M main
git push -uf origin main
```

## Integrate with your tools

- [ ] [Set up project integrations](https://git.lug.ustc.edu.cn/Mathison/teachur5_code/-/settings/integrations)

## Collaborate with your team

- [ ] [Invite team members and collaborators](https://docs.gitlab.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://docs.gitlab.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://docs.gitlab.com/ee/user/project/merge_requests/approvals/)
- [ ] [Set auto-merge](https://docs.gitlab.com/user/project/merge_requests/auto_merge/)

## Test and Deploy

Use the built-in continuous integration in GitLab.

- [ ] [Get started with GitLab CI/CD](https://docs.gitlab.com/ee/ci/quick_start/)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing (SAST)](https://docs.gitlab.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://docs.gitlab.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://docs.gitlab.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://docs.gitlab.com/ee/ci/environments/protected_environments.html)

***

# Editing this README

When you're ready to make this README your own, just edit this file and use the handy template below (or feel free to structure it however you want - this is just a starting point!). Thanks to [makeareadme.com](https://www.makeareadme.com/) for this template.

## Suggestions for a good README

Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection.

## Usage
Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README.

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.
>>>>>>> cff14a4edc65e9480040f16a08aa6be983738efd
