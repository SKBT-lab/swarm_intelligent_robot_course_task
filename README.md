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

### 安装以编译

```bash
# Clone the repository
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/SKBT-lab/swarm_intelligent_robot_course_task.git
cd ..
catkin_make
```


### 安装

```bash
# 克隆项目
git clone https://github.com/your-username/your-repo-name.git
cd your-repo-name

# 安装依赖
npm install

# 运行项目
npm start
