# 群体智能机器人技术及应用大作业

## 目录
- [写在前面](#特性)
- [快速开始](#快速开始)
- [安装](#安装)
- [使用指南](#使用指南)
- [API文档](#api文档)
- [开发](#开发)
- [贡献指南](#贡献指南)
- [许可证](#许可证)
- [联系方式](#联系方式)

## 写在前面
- 本次大作业共包含4个任务，选择其中一个完成。
- 本次作业的仿真平台基于ROS架构实现，需要同学们具备一定的ROS基础。
- 严禁更改该仿真程序中的相关参数及代码（即原始clone下来的所有代码均不能改动，仅能在此基础上填加任务实现的相关代码）。大家最终提交的代码作业将首先经过git diff查看原始代码是否被修改，一经发现，将酌情减分。对于仿真程序中可能存在的一些bug以及不合理的参数或配置，请第一时间和老师或助教反应，将根据实际情况统一调整。
- 4个任务的任务内容及启动方式将在下面分别介绍。

## 运行环境
ubuntu20.04(推荐，其他版本未做测试)

## 安装以编译

```bash
# Clone the repository
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/SKBT-lab/swarm_intelligent_robot_course_task.git
cd ..
catkin_make
```

## 基本接口
下面是四个任务都需要用到的一些基本接口：
任务涉及到的多个无人机按照uavX命名，X为无人机ID，需要执行任务的无人机ID从2号开始递增（1号为Task1中的NPC无人机）。
1. 无人机位姿话题：
   /uavX/sim/odom 类型：nav_msgs/Odometry
3. 无人机控制指令
   - 上层指令：/uavX/position_cmd  类型：quadrotor_msgs/PositionCommand
   - ...pathto/src/Swarm-Simulator/src/uav_simulator/so3_control/src/control_example.cpp提供了一个基本的用该指令控制无人机的例程，可用于参考。
   - 底层指令：/uavX/so3_cmd 类型： quadrotor_msgs/SO3Command
  默认情况下本程序通过...path/src/Swarm-Simulator/src/uav_simulator/so3_control/src/so3_control_nodelet.cpp中的简单的PID实现了由上层的PositionCommand向底层的SO3Command的计算，同学们也可以自行尝试采用更高级的控制方法以实现更好的效果（这部分代码允许更改）。
   - quadrotor_msgs的msg类型定义在...pathto/src/Swarm-Simulator/src/uav_simulator/Utils/quadrotor_msgs中，编写程序时记得在cmakelist中填加对包quadrotor_msgs的引用。

## 任务介绍
### 任务1 集群围捕
#### 任务要求
任务目标为对于一个高速移动的大机动目标，用多架无人机实现跟踪和包围。具体任务描述如下：
- 围捕目标为uav1，可通过/uav1/sim/odom实时获取它的位置坐标，但是严禁通过/uav1/position_cmd干扰其运动状态！打分的时候助教会检查源码。
- 控制6架无人机uav2-uav7，要求追踪过程中无人机之间不可发生相互碰撞。
- 要求的最终状态为6架无人机以一个固定的队形包围并随目标无人机一同移动，当所有无人机同目标无人机均保持在1-2m范围的距离内时，视作有效跟随。
- 要求有效跟随的累积时间大于目标无人机飞行总时间的70%。


# 克隆项目
git clone https://github.com/your-username/your-repo-name.git
cd your-repo-name

# 安装依赖
npm install

# 运行项目
npm start
