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
1. 无人机位姿话题：<br>
   /uavX/sim/odom 类型：nav_msgs/Odometry
2. 无人机控制指令:<br>
   - 上层指令：/uavX/position_cmd  类型：quadrotor_msgs/PositionCommand
     <br>...pathto/src/Swarm-Simulator/src/uav_simulator/so3_control/src/control_example.cpp提供了一个基本的用该指令控制无人机的例程，可用于参考。
   - 底层指令：/uavX/so3_cmd 类型： quadrotor_msgs/SO3Command
  <br>默认情况下本程序通过...path/src/Swarm-Simulator/src/uav_simulator/so3_control/src/so3_control_nodelet.cpp中的简单的PID实现了由上层的PositionCommand向底层的SO3Command的计算，同学们也可以自行尝试采用更高级的控制方法以实现更好的效果（这部分代码允许更改）。
   - quadrotor_msgs的msg类型定义在...pathto/src/Swarm-Simulator/src/uav_simulator/Utils/quadrotor_msgs中，编写程序时记得在cmakelist中填加对包quadrotor_msgs的引用。
3.碰撞判定<br>
根据常规无人机尺寸，下面对仿真中的碰撞进行规定：
   - 若无人机位置同最近的障碍物点云坐标距离小于0.2m，视作发生碰撞。
   - 若无人机之间的距离小于0.4m， 视作发生碰撞。
同学们可以在程序中自行判断是否碰撞，后续会更新判断碰撞的脚本用于最终验证。

## 任务介绍
### 任务1 集群围捕
#### 任务要求
任务目标为对于一个高速移动的大机动目标，用多架无人机实现跟踪和包围。具体任务描述如下：
- 围捕目标为uav1，可通过/uav1/sim/odom实时获取它的位置坐标，但是严禁通过/uav1/position_cmd干扰其运动状态！打分的时候助教会检查源码。
- 控制6架无人机uav2-uav7，要求追踪过程中无人机之间不可发生相互碰撞。
- 要求的最终状态为6架无人机以一个固定的队形包围并随目标无人机一同移动，当所有无人机同目标无人机均保持在1-2m范围的距离内时，视作有效跟随。
- 要求有效跟随的累积时间大于目标无人机飞行总时间的70%。
#### 启动方式
```bash
cd robot_ws/
source devel/setup.bash
roslaunch roslaunch so3_quadrotor_simulator task1.launch
```
运行后如下所示，rviz开启，其中运动的无人机即为要追踪的uav1:
![task1](https://github.com/SKBT-lab/swarm_intelligent_robot_course_task/blob/main/src/figure/task1.gif)
#### 注意
选择该题目的同学注意，在完成并编译追踪代码后，请将对应节点放置下图所示的位置并通过task1.launch来启动，以确保uav1和其他无人机是同时启动的
![task1](https://github.com/SKBT-lab/swarm_intelligent_robot_course_task/blob/main/src/figure/1-code.jpeg)
#### 提示
由于目标无人机机动性较强，直接基于其当前位置进行追踪难以保持长久的有效跟随，建议加入对uav1的运动预测。
### 任务2 集群穿越
#### 任务要求
任务目标为控制多架无人机无碰撞穿越障碍区域，飞行时间尽可能短，同时无人机彼此之间的距离需保持在一定范围内。具体任务描述如下：
- 控制6架无人机uav2-uav7穿越一个50m×50m的障碍区域，要求穿越过程不发生相互碰撞以及同障碍物之间的碰撞。
- 无人机之间的距离应保持在0.5m-5m之间：出于安全考虑，无人机之间不宜过近；出于通讯考虑，无人机之间不宜过远。
#### 启动方式
```bash
cd robot_ws/
source devel/setup.bash
roslaunch roslaunch so3_quadrotor_simulator task2.launch
```
运行后如下所示，rviz开启，无人机群从左上角出发，右下方的红点为目标点，集群整体飞行至目标点附近即可。
![task2](https://github.com/SKBT-lab/swarm_intelligent_robot_course_task/blob/main/src/figure/2-1.jpeg)
#### 注意
- 全局地图以点云的形式给出，话题为： /mock_map 类型为 sensor_msgs/PointCloud2
#### 提示
由于给出地图的是原始点云，需首先转为栅格或体素地图再进行进一步处理。
### 任务3 集群探索
#### 任务要求
任务目标为多架无人机在有限的FOV下，从地图中心出发，向四周探索，直至指定范围区域被所有无人机的历史FOV全覆盖。具体任务描述如下：
- 控制6架无人机uav2-uav7探索一个50m×50m的障碍区域，该区域的障碍物密度低于Task2，同样不允许发生任何形式的碰撞。
- 尽可能快地完成对整个场景的探索，最终探索率应大于95%。
- 假设无人机的相机竖直向下安装，对于一个（x,y,z）位置处的无人机，其FOV为地面上一个以（x，y）为圆心，半径r=0.5z的圆形范围。入下图所示：<br>
  ![task3](https://github.com/SKBT-lab/swarm_intelligent_robot_course_task/blob/main/src/figure/fov.gif)
#### 启动方式
```bash
cd robot_ws/
source devel/setup.bash
roslaunch roslaunch so3_quadrotor_simulator task3.launch
```
运行后如下所示，rviz开启，无人机群从中心区域出发，待探索范围为红色矩形围成的50mx50m区域。
![task3](https://github.com/SKBT-lab/swarm_intelligent_robot_course_task/blob/main/src/figure/3-2.jpeg)
#### 注意
- 为了降低该题目的复杂度，仿真程序提供了二维占据栅格地图的话题：/exploration_grid 类型为 nav_msgs/OccupancyGrid，其中：
  1. 障碍物栅格 为 100
  2. 已探索的自由栅格 为 0
  3. 未探索的自由栅格 为 -1
  该话题用于辅助探索算法，同时，在终端窗口中实时显示当前的探索率，以用来辅助判断和验证：
![task3](https://github.com/SKBT-lab/swarm_intelligent_robot_course_task/blob/main/src/figure/3-3.jpeg)
但是，对于避障环节可能需要的三维体素地图及其他形式的环境表述，仍需学生自行构建。
#### 提示
可参考已有的探索算法的基本流程，值得一提的是，常见的探索算法中无人机的FOV假定为一个扇形区域，而本作业中假定为圆形，这对问题进行了简化，因为不需要考虑yaw角的规划了。
# 克隆项目
git clone https://github.com/your-username/your-repo-name.git
cd your-repo-name

# 安装依赖
npm install

# 运行项目
npm start
