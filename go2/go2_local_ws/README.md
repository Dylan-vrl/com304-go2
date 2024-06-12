**Warning:** Our code may become invalid or be put in an unusable state due to changes required to support the QRE SDK. For example, if the SDK changes the names of certain ROS topics, you will have to adapt the code accordingly.

**Warning:** The QRE SDK is a private repository available only for clients of [MYBOTSHOP.de](https://www.mybotshop.de/), to get access to the repository you must get in contact with them.

**Warning:** Do not run `apt update` or `apt upgrade` under any circumstances in the robot as this will break most of the functionalities of the SDKs.

## Quick start
You should clone both this repository and the [foxy-nvidia branch of the QRE repository](https://github.com/MYBOTSHOP/qre_go2/tree/foxy-nvidia) into the Go2's onboard NVIDIA Jetson Orin, to which you can SSH (see [GO2 Network Interface](https://www.docs.quadruped.de/projects/go2/html/go2_driver.html#go2-network-interface) for more details).

1. Connect to the robot via SSH over LAN by following the [documentation](https://www.docs.quadruped.de/projects/go2/html/go2_driver.html#go2-network-interface).
2. Connect the robot to the Internet:
    * We used a Wi-Fi USB module with USB ID `0bda:c820`, which uses the `RTL8821CU` chipset
    * To set up this USB module, we cloned and built the `https://github.com/brektrou/rtl8821cu.git` repo which provides the Linux driver for this chipset to our local machine, then copied it to the Jetson board (with `scp`)
    * Unplug and re-plug in the Wi-Fi USB module so it is recognized
    * We used `nmcli` to connect to wifi:
    ```bash
    sudo nmcli dev wifi list # lists available networks
    sudo nmcli --ask dev wifi connect <SSID> # connect to <SSID>
    ```
    * Once connected to Wi-Fi, one can now easily clone and install other repositories, package updates, or connect via SSH over Wi-Fi for convenience
3. Clone the `foxy-nvidia` branch of the `qre_go2` repository onto the Jetson
4. Build and install the repository:
    * On our first attempt, we naively ran the `go2_install.bash` script which runs `apt-get` to install and update many packages. Unfortunately, this broke our existing codebase because our code targets ROS Humble, and the on-board Jetson runs ROS Foxy.
    * To avoid this issue, we repeatedly attempted building only the ROS packages required for our use-case (such as `go2_depth_camera` to support the d435i depth camera, through `colcon build --symlink-install --packages-select go2_depth_camera`), and only ran `apt-get install` when an error occurred. This process was a little tedious, but by avoiding a full-blown `apt update` or an `apt install` of many packages, and instead limiting changes to only required dependencies, we were able to maintain functionality of our existing codebase alongside the new QRE repo.
5. Clone and build this repository onto the NVIDIA Jetson board
6. Adapt [`startup.bash`](startup.bash) with the correct paths (by hard-coding them or providing the `QRE_GO2_PATH` and `COM304_GO2_PATH` environment variables) and `source` it. It will launch the nodes necessary for autonomous navigation, control, and sensor input. If you face performance issues, see [troubleshooting](#troubleshooting).
7. In a second terminal, start the autonomous navigation (see [Usage/Autonomous run](../go2_wifi_ws/README.md#autonomous-run))

## Usage
This workspace shares the same functionalities as the `go2_wifi_ws`. Take a look at its [Usage](../go2_wifi_ws/README.md#usage) section for details.

## Troubleshooting

- _The robot receives the move or rotate commands but it doesn't move_\
  If you lifted up the robot by hand or stopped by killing the process, the robot may be in an invalid state. To reset it, just publish on the `/stop` topic. If it doesn't work, restart the robot.
- _The robot is very laggy and the camera frames do not update frequently enough_\
    You may want to create your own launch file, as minimalist as possible, launching only the necessary nodes for your task. The nodes we need for our task are:
    ```bash
    go2_base go2_highroscontrol
    go2_base go2_statepublisher
    realsense2_camera realsense2_camera_node
    com304_go2 go2_control_node
    com304_go2 go2_autonomous_node
    ```
    You could also try to deactivate the built-in GO2's topics publication.
