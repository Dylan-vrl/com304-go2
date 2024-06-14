# Isaac SIM task
This folder contains the source code for the Isaac lab task. 
It is at a state where the environment is almost ready but the training pipeline is not yet functional as it 
needs some tweaks.

## Simulator
We are using [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) (previously known as [Isaac Orbit](https://isaac-orbit.github.io/)) 
which is a wrapper for [Isaac sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) with RL oriented features.
The simulator was developed by NVIDIA and provides high performance and fidelity simulation with real time ray tracing,
allowing us to run parallelized environments exclusively on GPU.

### Requirements 
- NVIDIA RTX GPU with CUDA 11.0 or higher and at least 12GB of VRAM
- Ubuntu 22.04 or higher
- High-ish end CPU

### Examples of possible tasks
The simulator allows for native ROS2 support and provides a set of sensors, including RGBD cameras, IMU, and LIDAR to mention a few.
While learning how to use the simulator, we have been able to create a simple environment with a GO2 robot controlled using 
a trained low level policy and ROS2 teleop, we were also able to get sensor data such as Lidar (note: these visible points are for debugging).

This is shown in the video below (click).<br/>
[![lidar_go2](https://img.youtube.com/vi/Dx7pLRo_t60/0.jpg)](https://www.youtube.com/watch?v=Dx7pLRo_t60)

It is also possible to create more complex tasks such as the one shown in the video below where an agent has to navigate 
through rough randomized terrain using force and lazer sensors as shown in the video below (click). <br/>
[![rough_terrain](https://img.youtube.com/vi/uhK3vNfGLug/0.jpg)](https://www.youtube.com/watch?v=uhK3vNfGLug) 

## Our task
### Scene
We started by creating a simple room that will be used as the environment for the task. It has lighting and multiple groups of meshes 
for example ceiling, floor, wall_wood, wall_reg... which will allow for coherent randomization when we want to train (for example materials and textures). <br/>
![basic_room](media/basic_room.png)

We also created a model for the lab room. <br/>
<img src="media/lab_top.png" width="49%"/> <img src="media/lab_inside.png" width="49%"/> <br/><br/>
We can then spawn a model for the robot that is modeled by a cuboid of same dimensions as the Go2 with an RGBD camera 
attached to it and a red ball. We made it so that they spawn in random locations in the room each time the environment is reset.
[(scene config)](targetnav/configs/scene.py)<br/>
<img src="media/basic_setup.png" width="49%"/> <img src="media/robot_pov.png" width="49%"/> <br/><br/>

### Control
We were able to control out robot using both distance based and velocity based commands, the below shows the robot moving
using distance commands (KI controller), however we opted for velocity commands in the end as they are closer to the actual robot's API. 
[(actions config)](targetnav/configs/action.py) (video) <br/>
[![distance_control](https://img.youtube.com/vi/K6_RPhuBeDU/0.jpg)](https://www.youtube.com/watch?v=K6_RPhuBeDU)

### Reinforcement learning environment
The observations, rewards and terminations of the environment can be found in the [base_env_setup](targetnav/base_env_setup.py) file. <br/><br/>
The input to the agent is a 4D tensor of shape (128, 128, 4) (rgbd), which is perfect for a CNN based policy. The group 'sim' is used internally for 
example for reward generation and terminations. [observations mdp](targetnav/mdp/observations.py) <br/>
![observations](media/observations.png) <br/><br/>
The rewards are an intermediary reward for being close to the ball (l2 distance smaller than a threshold) and a final 
reward for being oriented correctly.
Negative rewards are also used to avoid certain situations like long runs and wall slides etc... A termination command 
has not yet been taken into consideration in the rewards design but is a good idea for real robots. [rewards mdp](targetnav/mdp/rewards.py)<br/>
![rewards](media/rewards.png) <br/><br/>
The rest of the Managers look like the following: <br/> 
<img src="media/event_term.png" width="49%"/> <img src="media/comm_actions.png" width="49%"/>  <br/><br/>

### Training pipeline
Training in isaac lab is done through wrapping the environment in multiple wrappers. Starting with registering it as a 
[gymnasium](https://gymnasium.farama.org/index.html) environment (previously known as OpenAI gym). This is done in the 
[__init__.py](targetnav/__init__.py) file. <br/>
Afterward, the environment can be wrapped for video recording, logging, etc... <br/>
And finally the environment is wrapped in a training framework such as [Stable Baselines3](https://stable-baselines3.readthedocs.io/en/master/index.html)
or [rl_games](https://github.com/Denys88/rl_games). The configurations can be found in the [agents](targetnav/agents) folder and 
an example pipeline for sb3 in the [train.py]() file.<br/>

We can train a lot of instances in parallel (order of 1000s depending on amount of graphics memory and environment) while 
using headless mode which disables full visual rendering and only keeps the required buffers in VRAM and the training 
happens on the order of 10s of thousands of FPS. <br/>

Below is a video example with 8 environments with rendering on. <br/>
[![envs](https://img.youtube.com/vi/4SBhNwrkMtw/0.jpg)](https://www.youtube.com/watch?v=4SBhNwrkMtw) <br/>

### Results
We were not able to get a final model due to time constraints, but we tried to train a model using the PPO algorithm 
with both MLP and CNN architectures which have configs defined in the [agents](targetnav/agents) folder.

Using tensorboard to visualize the training, we can see that the model is minimizing the loss but the rewards were not 
increasing as expected, instead they were decreasing with time which needs further investigation. <br/>
<img src="media/graphs.jpg" width="49%"/> <img src="media/graphs_rollout.jpg" width="49%"/>  <br/>

We also faced last minute problems with mesh collisions which were not working properly which led to performance and 
scaling problems while training. <br/>
![incompatible mesh](media/incompatible_mesh.png) <br/>
