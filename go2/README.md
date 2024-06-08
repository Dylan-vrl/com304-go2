Every step described below has only been tested on Ubuntu 22.04 LTS with the `humble` ROS distribution.

## Structure
This directory contains all the code related to controlling the robot. It is separated in three subfolders, which we advice you to explore in this order:

- [`go2_ethernet`](go2_ethernet): Contains code to  **manually** control the robot over an ethernet connection. You don't need to interact with the robot through ROS, a C++ wrapper SDK is used instead.
- [`go2_wifi_ws`](go2_wifi_ws): Contains code to manually and autonomously control the robot over WiFi.
- [`go2_local_ws`](go2_local_ws): Contains code to manually and autonomously control the robot locally on its onboard computer.

## How to choose your workspace
- [`go2_ethernet`](go2_ethernet) is a great starting point to understand the robot available commands and topics. However, as it can only be used through ethernet, it is not very suitable as a final solution.
- [`go2_wifi_ws`](go2_wifi_ws) runs the nodes on your local machine, which means you need a Linux machine. This leaves more control over the code and generally better performance. It does not require any extra piece of hardware other than the robot and your computer. However you won't have access to GO2's depth camera data. The community-made SDK that is used is less exhaustive than the QRE one.
- [`go2_local_ws`](go2_local_ws) runs the nodes on the robot machine. This provides greater control over the topics and runs on any machine that can SSH into the robot. The QRE SDK provides more functionalities and is professionnally maintained. However this requires a more complex setup phase and you need a USB dongle.