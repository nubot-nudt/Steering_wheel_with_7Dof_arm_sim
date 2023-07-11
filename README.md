# Steering_wheel_with_7Dof_arm_sim
Project of Steering wheel in coppeliasim.

这个Project提供了2个Ros功能包nubot_msgs、r7_auto_sim；4个仿真结果视频；1个coppeliasim仿真场景文件Vrep_steering_wheel_with_7Dof.ttt；1个插件simExtROS_for_Vrep_4.1。



运行例程前，请确保coppeliasim为4.1版本，并安装与ROS通信插件。

### 载入仿真环境

运行coppeliasim仿真环境，通过file-open sense选项打开给定的.ttt场景文件，点击run按钮开始仿真

预设的4电机角度为90°，采用130°、155°，请双击joint1-7，参考下表设置各自角度值

|      | Joint1 | Joint2 | Joint3 | Joint4 | Joint5 | Joint6 | Joint7 |
| :--: | :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| 90°  |   0    |   0    |  -45   |  -90   |   0    |   0    |   0    |
| 130° | -23.65 |  -21   | -42.13 |  -50   |  1.19  |  0.66  |  8.33  |
| 155° | -36.5  | -29.6  | -38.6  |  -25   | -3.23  |  1.01  | 18.29  |



### 启动控制节点

1.新建工作空间，使用catkin_make命令编译给定的nubot_msgs、r7_auto_sim这两个功能包

2.source 此工作空间

```
source  \devel\setup.bash 
```

3.运行r7_auto_sim这个节点即开始控制

```
rosrun r7_auto_sim r7_auto_sim
```

