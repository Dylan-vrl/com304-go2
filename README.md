# Autonomous target navigation using Unitree Go2

This project aims to deploy an autonomous target navigation model onto the Unitree Go2 robot platform.

The model has been trained using reinforcement learning in Habitat in indoor environments randomly populated with obstacles in search of a pre-defined target (a pink ball). It is deployed on the real agent using a ROS2 environment. To gather data and send commands to the Unitree Go2, we use the [Quadruped Robotics Go2 SDK](https://github.com/MYBOTSHOP/qre_go2/tree/foxy-nvidia), dedicated to their customers.

We also explored training our modle on the Isaac simulator. By lack of time, we couldn't produce a final model. You can still find all the related code in the [isaac folder](isaac).

- [go2](go2) folder contains the source code to control the robot manually and autonomously through ethernet, WiFi or locally on the robot computer
- [isaac](isaac) folder contains the source code for the Isaac Lab task definition
