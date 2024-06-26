# ROS 和 Vue 通信演示

## 目的

这个项目展示了如何使用 Vue 3 前端与 ROS（机器人操作系统）进行通信。通过该演示，用户可以使用网页上的按键来控制仿真环境中的 TurtleBot3 机器人，并实时查看机器人的位置和速度信息。

## 技术和版本

- **ROS（机器人操作系统）**：Noetic
- **rosbridge_suite**：0.11.13
- **roslibjs**：1.3.0
- **Vue**：3.x
- **Node.js**：14.x 或更高版本

## 使用方法

### 前提条件

1. 安装 ROS Noetic 并配置环境。
2. 安装 `rosbridge_suite`：

   ```bash
   sudo apt-get install ros-noetic-rosbridge-server
   ```

3. 安装 TurtleBot3 模拟包：

   ```bash
   sudo apt-get install ros-noetic-turtlebot3-simulations
   ```

### 启动 ROS 和仿真环境

1. 启动 rosbridge 服务器（这会自动启动 `roscore`）：

   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

2. 启动 Gazebo 仿真环境：

   ```bash
   export TURTLEBOT3_MODEL=burger
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```

### 启动前端项目

1. 克隆这个仓库并进入项目目录：

   ```bash
   git clone https://github.com/jjhhyyg/ros-vue-demo.git
   cd ros-vue-demo
   ```

2. 安装依赖：

   ```bash
   npm install
   ```

3. 启动 Vue 开发服务器：

   ````bash

   ```npm run dev
   ````

### 控制机器人

在浏览器中打开 Vue 应用后，你可以通过以下按键控制机器人：

- **W**：增加线速度
- **X**：减少线速度
- **A**：增加角速度
- **D**：减少角速度
- **S** / **Space**：停止机器人

### 实时显示

- **Robot Position**：显示机器人的当前位置 (x, y, z)。
- **Current Speed**：显示机器人的当前线速度 (linear) 和角速度 (angular)。

通过上述步骤，你可以成功地启动并运行该演示，体验使用 Vue 前端与 ROS 进行通信和控制机器人的全过程。
