# Usage

Warning, this doesn't work yet! Delays make the move for a certain distance algorithm fail to reach its goal.

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

=== Terminal 1 (driver) ===

The driver is part of the sdk and publishes the data (LiDAR, camera, position, IMU) and offers topic to control the robot (`/cmd_vel` for example)
```bash
export ROBOT_IP="Your robot ip"
source install/setup.bash
ros2 launch com304_go2 driver.launch.py
```

=== Terminal 2 (control) ===

The control is made by ourselves. It publishes to control topics (`/cmd_vel`, ...) and offers custom topics for ease of use (`/move` and `/rotate`)
```bash
source install/setup.bash
ros2 launch com304_go2 control.launch.py
```


=== Terminal 3 (interactive) ===
Publish to the `/move` and `/rotate` topics to control the robot.
```bash
source install/setup.bash
ros2 topic pub /move com304_interfaces/msg/Move "{x: 0.3, y: 0}"
ros2 topic pub /rotate com304_interfaces/msg/Rotate "{yaw: 1.57}"
```
