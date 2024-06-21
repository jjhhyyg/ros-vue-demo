# ROS and Vue Communication Demo

## Purpose

This project demonstrates how to communicate between a Vue 3 frontend and ROS (Robot Operating System). With this demo, users can control a simulated TurtleBot3 robot using buttons on a web page and view the robot's position and speed information in real time.

## Technologies and Versions

- **ROS (Robot Operating System)**: Noetic
- **rosbridge_suite**: 0.11.13
- **roslibjs**: 1.3.0
- **Vue**: 3.x
- **Node.js**: 14.x or higher

## Usage

### Prerequisites

1. Install ROS Noetic and set up the environment.
2. Install `rosbridge_suite`:
    ```bash
    sudo apt-get install ros-noetic-rosbridge-server
    ```
3. Install TurtleBot3 simulation package:
    ```bash
    sudo apt-get install ros-noetic-turtlebot3-simulations
    ```

### Start ROS and Simulation Environment

1. Start the rosbridge server (this will automatically start `roscore`):
    ```bash
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

2. Start the Gazebo simulation environment:
    ```bash
    export TURTLEBOT3_MODEL=burger
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```

### Start the Frontend Project

1. Clone this repository and navigate to the project directory:
    ```bash
    git clone https://github.com/jjhhyyg/ros-vue-demo.git
    cd ros-vue-demo
    ```

2. Install dependencies:
    ```bash
    npm install
    ```

3. Start the Vue development server:
    ```bash
    npm run dev
    ```

### Control the Robot

Once the Vue application is opened in the browser, you can control the robot using the following keys:

- **W**: Increase linear speed
- **X**: Decrease linear speed
- **A**: Increase angular speed
- **D**: Decrease angular speed
- **S** / **Space**: Stop the robot

### Real-Time Display

- **Robot Position**: Displays the current position of the robot (x, y, z).
- **Current Speed**: Displays the current linear speed (linear) and angular speed (angular) of the robot.

Follow these steps to successfully start and run this demo, experiencing the full process of using a Vue frontend to communicate with ROS and control a robot.

---

[中文文档](./README.zh.md)
