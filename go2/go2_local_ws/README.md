**Warning:** Our code may become invalid due to changes in the QRE SDK we use. For example, if they change some topics name, you will have to adapt the code accordingly.

**Warning:** The QRE SDK is a private repository available only for their clients, you should ask them access for it.

**Warning:** Do not run `apt update` or `apt upgrade` under any circumstances in the robot as this will break most of the functionalities of the SDKs.

## Quick start
You should clone both this repository and the [QRE one](https://github.com/MYBOTSHOP/qre_go2/tree/foxy-nvidia) into the GO2's Nvidia board, to which you can SSH (see [GO2 Network Interface](https://www.docs.quadruped.de/projects/go2/html/go2_driver.html#go2-network-interface) for more details).
1. Connect the robot to Internet:\
    **TODO by Hod**
2. Connect to the robot through SSH
3. Clone the `qre_go2` repository, switch to the `foxy-nvidia` branch
4. Install the QRE repository:\
    **TODO by Hod** (not running the go2_install.bash but individual commands)
5. Build the QRE repository
5. Clone and build this repository
6. Adapt [`startup.bash`](startup.bash) with the correct paths and run it. It will launch the nodes necessary for autonomous navigation. If you face performance issues, see [troubleshooting](#troubleshooting).
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