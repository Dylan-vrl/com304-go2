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

The control is made by ourselves. It publishes to control topics (`/cmd_vel`, ...) and offers custom topics for ease of use (`/move` and `/rotate`)
```bash
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: 0.3, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: 1.57}"
ros2 topic pub --once /stop std_msgs/msg/Empty
ros2 topic pub --once /save_camera std_msgs/msg/Empty
```
