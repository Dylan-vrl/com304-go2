source $QRE_GO2_PATH/install/setup.bash
source $COM304_GO2_PATH/install/setup.bash
ros2 launch go2_base base.launch.py
ros2 launch go2_depth_camera system.launch.py
ros2 launch com304_go2 autonomous.launch.py