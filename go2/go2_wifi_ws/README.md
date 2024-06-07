For more details about the installation process, take a look at the [`go2_ros2_sdk repository`](https://github.com/abizovnuralem/go2_ros2_sdk). 

It is assumed you have both [`rust`](https://www.rust-lang.org/tools/install) and [ros2](https://docs.ros.org/en/humble/Installation.html) installed. 

Our code may become invalid due to changes in the SDK we use. For example, if they change some topics name, you will have to adapt the code accordingly.

## Quick start

1. Make sure the [`go2_ros2_sdk`](https://github.com/abizovnuralem/go2_ros2_sdk) components are up-to-date by running [`reclone_sdk.sh`](reclone_sdk.sh). This will clone the repository and all its submodules and copy the files into `src`
2. Using the app, connect the robot to the same WiFi network as your computer and get its IP either through the app (Device -> Data -> Automatic Machine Inspection, (look for STA Network: wlan0)) or by checking the clients connected to your router
3. Build the workspace 
    ```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    ```
    If you already built it once, you don't need to rebuild all packages everytime. Especially the [`go2_video`](src/go2_video/) one. Use `colcon build --packages-select` to only select the modified packages instead.
4. Launch the SDK nodes
    ```bash
    source install/setup.bash
    export ROBOT_IP="<robot_ip>"
    export CONN_TYPE="webrtc"
    ros2 launch go2_robot_sdk robot.launch.py
    ```

## Structure

Here is a list of the important topic in our workspace:
```bash
## Our topics (main interaction API) ##
/move # Starts moving the robot by a specific distance (queue system)
/rotate # Starts rotating the robot by a specific angle (queue system)
/stop # Stops any ongoing movement (clears the queue)
/goal_reached # Signals that the current goal has been reached
/save_camera # Save the current camera feed into a JPEG image
/start # Starts the autonomous navigation

## SDK topics (used underneath by our topics) ##
/go2_states # Publishes information about the robot state, including position
/imu # Publishes the robot IMU
/cmd_vel # Transmits the velocity commands to the robot
/command # Transmits pre-defined actions to the robot (sit, roll, stop, ...)
```
- [`com304_go2`](src/com304_go2/): Contains all our custom code for manual discrete control, autonomous RL agent and image processing
  - [`go2_control_driver_node`](src/com304_go2/com304_go2/go2_control_driver_node.py): Lightweight copy of the [`go2_driver_node`](src/go2_robot_sdk/go2_robot_sdk/go2_driver_node.py) from the sdk, exposing only the topics we use to use less resources. Publishes the robot position and rotation and forwards upcoming commands to the robot
  - [`go2_control_node`](src/com304_go2/com304_go2/go2_control_node.py): Exposes the final user API with `/move`, `/rotate` and `/stop` topics. Converts distance-based requests to velocity-based commands for the robot.
  - [`go2_autonomous_node`](src/com304_go2/com304_go2/go2_autonomous_node.py): Handles autonomous navigation based on a habitat RL model with discrete action space
  - [`models` folder](src/com304_go2/models/) and [`config` folder](src/com304_go2/config/) contains the checkpoints and config for the trained navigation and RGB-to-depth models to use
- [`com304_interfaces`](src/com304_interfaces/): Defines the custom messages our topics use
- Other folders are part of the SDK and are prone to changes. Check their GitHub for more details

## Usage

### Manual run
=== Terminal 1 ===
```bash
export ROBOT_IP="<robot_ip>"
export CONN_TYPE="webrtc"
source install/setup.bash
ros2 launch com304_go2 movement.launch.py
```

=== Terminal 2 ===
```bash
source install/setup.bash
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: 0.3, y: 0}" # meters
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: 1.57}" # radians
ros2 topic pub --once /stop std_msgs/msg/Empty
```

### Autonomous run
=== Terminal 1 ===
```bash
export ROBOT_IP="<robot_ip>"
export CONN_TYPE="webrtc"
source install/setup.bash
ros2 launch com304_go2 autonomous.launch.py
```

=== Terminal 2 ===
```bash
source install/setup.bash
ros2 topic pub --once /stop std_msgs/msg/Empty # Put the robot in valid state
ros2 topic pub --once /start std_msgs/msg/Empty # Start navigations
```


## Autonomous navigation
There's multiple points to consider regarding autonomous navigation using a trained model. First, as it is, the model's action space should be discrete:

0. Stop
1. Move forward
2. Turn left
3. Turn right

Its observation space should expect RGB and depth data. Note however that the depth data is estimated from RGB using [`monodepth2`](https://github.com/nianticlabs/monodepth2). This method has limitations, in particular in empty rooms or if the robot is facing a wall: without enough information the depth estimate will be bad. If you need higher precision depth information, consider using [go2_local_ws](../go2_local_ws/) instead.

The checkpoint (`.pth`) files you provide in the `models` folder should only contain the content of the `state_dict` key of your model. To convert your model to the correct format you can use the dedicated [`ckpt_to_model_only.py`](../../ckpt_to_model_only.py) script.


## Troubleshooting

- _The robot receives the move or rotate commands but it doesn't move_\
  If you lifted up the robot by hand or stopped by killing the process, the robot may be in an invalid state. To reset it, just publish on the `/stop` topic. If it doesn't work, restart the robot.