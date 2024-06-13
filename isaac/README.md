# Isaac SIM task
This folder contains the source code for the Isaac lab task. 
It is at a state where the environment is almost ready but the training pipeline is not yet functional as it 
needs some tweaks.

## Simulator
We are using [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) (previously known as [Isaac Orbit](https://isaac-orbit.github.io/)) 
which is a wrapper for [Isaac sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) with RL oriented features.
The simulator was developed by NVIDIA and provides high scalability, performance and fidelity simulation, allowing us to run 
parallelized environments exclusively on GPU.

### Requirements 
- NVIDIA RTX GPU with CUDA 11.0 or higher and at least 12GB of VRAM
- Ubuntu 22.04 or higher
- High-ish end CPU

### Examples of possible tasks
The simulator allows for native ROS2 support and provides a set of sensors, including RGBD cameras, IMU, and LIDAR to mention a few.
While learning how to use the simulator, we have been able to create a simple environment with a GO2 robot controlled using 
a trained low level policy and ROS2 teleop, we were also able to get sensor data such as Lidar (note: these visible points are for debugging). <br/>
[![lidar_go2](https://img.youtube.com/vi/Dx7pLRo_t60/0.jpg)](https://www.youtube.com/watch?v=Dx7pLRo_t60)

It is also possible to create more complex tasks such as the one shown in the video below where an agent has to navigate 
through rough randomized terrain using force and lazer sensors. <br/>
[![rough_terrain](https://img.youtube.com/vi/uhK3vNfGLug/0.jpg)](https://www.youtube.com/watch?v=uhK3vNfGLug) 

## Our task
### Simulation environment

### Reinforcement learning implementation

