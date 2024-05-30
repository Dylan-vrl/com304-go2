# Structure

## `com304-go2` package
It contains almost all our code. It has 4 ros nodes and many python files (for the habitat model and the rgb-to-depth model)

### `go2_control_driver_node` node
A lightweight version of the `go2_driver_node` from the `go2_ros2_sdk` to keep only the functionalities we're using.

### `go2_control_node` node
Subscribes to the `/move`, `/rotate` and `/stop` topics to transmit them adequately to the `go2_control_driver_node` using the `/cmd_vel` and `/command` topics (which are exposed by the `go2_ros2_sdk` by default.

Publishes to `\goal_reached` when it reaches its goal (either rotation goal or translation goal).

Keeps a queue of actions upon receiving commands. It executes the next action in the queue on goal reached.

### `go2_camera_node` node (debug only node)
/!\ Debug only node /!\

Subscribes to the `/go2_camera/color/image` topic to get the real-time RGB camera data. Updates an internal `last_image` variable to the latest image frame. Also subscribes to the `/save_camera` topic to save the latest frame to jpeg format.

### `go2_autonomous_node` node
Subscribes to `/goal_reached` and `/start`. Upon publication, prompt its internal habitat model for the next action to take, based on the last image frame, which is updated upon `/go2_camera/color/image` publication (exactly as the `go2_camera_node`).

Once the next action is chosen, publishes to `/move`, `/rotate` and `/stop` accordingly.

Start the autonomous navigation by publishing on the `/start` topic:
```bash
ros2 topic pub --once /start std_msgs/msg/Empty
```

### `custom_agent` script
Wrapper around the habitat model, which exposes an `act(observations)` function to decide the next action to take (represented by a number)
- 0: Stop
- 1: Move forward
- 2: Turn left
- 3: Turn right

`observations` is a dict with `rgb` and `depth` keys. If the `depth` key is not provided, then it is estimated from the rgb data using `monodepth2`.

### launch directory

#### `autonomous.launch.py`
Launch the nodes needed for autonomous control.

=== Terminal 1 ===

```bash
export ROBOT_IP="Your robot ip"
source install/setup.bash
ros2 launch com304_go2 autonomous.launch.py
```

=== Terminal 2 ===

```bash
source install/setup.bash
ros2 topic pub --once /start std_msgs/msg/Empty
```

#### `movement.launch.py`
Launch the nodes needed for movement only.

=== Terminal 1 ===

```bash
export ROBOT_IP="Your robot ip"
source install/setup.bash
ros2 launch com304_go2 movement.launch.py
```

=== Terminal 2 ===

```bash
source install/setup.bash
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: 0.3, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: 1.57}"
ros2 topic pub --once /stop std_msgs/msg/Empty
```

#### `robot.launch.py`
Launch the nodes needed for movement and camera.

=== Terminal 1 ===

```bash
export ROBOT_IP="Your robot ip"
source install/setup.bash
ros2 launch com304_go2 robot.launch.py
```

=== Terminal 2 ===

```bash
source install/setup.bash
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: 0.3, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: 1.57}"
ros2 topic pub --once /stop std_msgs/msg/Empty
ros2 topic pub --once /save_camera std_msgs/msg/Empty
```

## `com304-interfaces` package
Creates the `com304_interfaces/msg/Move` and `com304_interfaces/msg/Rotate` message types.

## `go2_robot_sdk`, `go2_video`, `go2_interfaces` packages
They form the `go2_ros2_sdk`.

# Usage

Make sure you have rust and ros2 installed

=== To run only once ===

```bash
sudo apt install python3-pip clang
pip install -r go2_ws/src/requirements.txt
```

=== To run every time you modify a script (rebuild) ===

```bash
source /opt/ros/humble/setup.bash
cd go2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

=== Terminal 1 ===

```bash
export ROBOT_IP="Your robot ip"
source install/setup.bash
ros2 launch com304_go2 robot.launch.py
```

=== Terminal 2 ===

```bash
source install/setup.bash
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: 0.3, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: 1.57}"
ros2 topic pub --once /stop std_msgs/msg/Empty
ros2 topic pub --once /save_camera std_msgs/msg/Empty
```
