# Usage

Warning, this doesn't work yet!

=== Terminal 1 ===
source /opt/ros/humble/setup.bash
cd go2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build

source install/setup.bash
ros2 launch com304_go2 driver.launch.py

=== Terminal 2 ===
source install/setup.bash
ros2 launch com304_go2 control.launch.py

=== Terminal 3 ===
ros2 topic pub /move com304_interfaces/msg/COM304Move {x: 0.3, y: 0}
ros2 topic pub /rotate com304_interfaces/msg/COM304RRotate {yaw: 1.57}
