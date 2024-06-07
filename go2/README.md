Every step described below has only been tested on Ubuntu 22.04 LTS

This directory contains all the code related to controlling the robot. It is separated in three subfolders:

- [`go2_ethernet`](go2_ethernet): Contains code to  **manually** control the robot over an ethernet connection. It is a great starting point to understand the robot available commands and topics. You don't need to interact with the robot through ROS, a C++ wrapper SDK is used instead.