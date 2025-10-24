-----2024------

```
5-27:
    1.修改urdf模型的world关节位置，由原来base_link处改为body_base_link处
```

```
5-28
    1.left与right.json添加back操作的两个姿态
```

```
5-29
	1.将研磨工序进行整合，打包到前缀Grind_的文件中
    2.大模型指令（研磨的指令），均加Grind_
    3.将最新研磨动作放置在Grind_capsulation.cpp中，内置有研磨轨迹记录与执行程序
```

```
6-07
	1.AB_manipulator.cpp改为约束拉伸
    2.robot_arm.cpp添加约束拉伸api
    3.拉伸的两个姿态进行更改
    4.manipulator功能包依赖ompl库
```

```
6-13
	1.universal_tools.cpp添加Kinematics_tools类，该类包括单/双臂，添加Constraint_tools类，该类包括单臂，双臂暂未添加
    2.Z_testfield.cpp测试右臂x轴不动约束，使用方法为ompl,并可以进行约束规划
    3.headfile.h中添加ompl头文件
```

```
6-13
	1.universal_tools.cpp添加双臂约束DualArm_Constraint类;
    2.omplPathToMoveItPath()函数的输入变量更改为moveit::planning_interface::MoveGroupInterfacePtr mgptr;
    3.Z_testfield.cpp测试双臂x轴不动约束，目前out[0]约束r.x,out[1]约束l.x,对应写了其雅可比矩阵.
```

```
6-19
	1.双臂约束DualArm_Constraint类进行微调;
    2.jacobian_formula()与constraint_formula()进行调整;
    3.universal_tools.cpp添加Constraint_S、M、H三个函数
```

```
6-24
	1.完成机械臂闭链约束，并对universal_tools.cpp中的jacobian_formula()与constraint_formula()进行闭链完善；
    2.对universal_tools.cpp中的Constraint_S进行调整，即转置;
    3.universal_tools.cpp添加CloseChain_cst_move()函数;
    4.ZZ_store_testcode.cpp中将测量双臂末端相对位姿的程序投放进去，以作保存便于后续使用。
```

```
6-26
	1.闭链约束存在些许bug（位置约束要在公式上整体乘-1）;
    2.位置约束在不知名bug下，可以运行;
    3.姿态约束不能奏效，猜测可能与该bug有关，尽管约束应该没问题，大概是求其姿态雅克比存在问题。
```

```
6-27
	1.闭链约束存在些许bug（位姿约束要在公式上整体乘-1）;
    2.大概是约束容差值的原因吧。
```

```
7-01
	1.universal_tool.cpp添加Get_LtargetPose(),通过当前相对姿态，获得移动后，根据右臂姿态推出左臂姿态;
    2.更改位置与姿态约束方程的前置系数（位置*10，姿态/10，统一量标）;
    3.robot_arm.cpp修改Gel_strech_motion()，即凝胶拉伸动作api。
```

```
7-07
	1.将该工程简约为凝胶任务，剔除研磨任务的程序。
```

```
8-18
	1.添加follow_masterarm的函数;
    2.更改CloseChain_cst_move函数;
    3.添加轨迹yaml读取与储存函数.
```