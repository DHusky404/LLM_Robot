# 基于大模型的智能寻物机器人

## 环境配置

1. 安装ROS：

​	项目使用的系统版本是**Ubuntu20.04**，对应的ROS版本为**Noetic Ninjemys**；

2. 创建一个ROS项目并进行配置：

​	创建工作空间：

```linux
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

​	设置环境变量

```
gedit ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

3. 获取源码：

```
cd ~/catkin_ws/src/
git clone https://github.com/DHusky404/LLM_Robot.git
```

4. 安装依赖项：

```
cd ~/catkin_ws/src/scripts
./install_deps.sh
```

## 快速开始

1. 开启rviz和Gazebo，初始化仿真环境和机器人

```
roslaunch nav_pkg nav_with_image.launch 
```

2. 获取摄像头传感器数据

```
rosrun rqt_image_view rqt_image_view 
```

3. 配置大模型的参数文件（包括api-key，base_url和模型名称）

```
gedit ~/catkin_ws/src/llm_pkg/config/model.yaml
```

4. 启动大模型通信节点

```
roslaunch llm_pkg llm.launch
```

5. 发送指令

```
rosrun llm_pkg input_node 你好
```

