## Quick start
1. Establish ethernet connection\
    Follow the instructions in the [GO2 Network Interface](https://www.docs.quadruped.de/projects/go2/html/go2_driver.html#go2-network-interface) of the robot's documentation.
2. Build the [`unitree_sdk2`](ext/unitree_sdk2/) project
    ```bash
    cd ext/unitree_sdk2
    sudo ./install.sh
    mkdir build
    cd build
    cmake ..
    make
    ```
3. Run an example script
    ```bash
    sudo ./sportmode_test <network_interface_name>
    ```

Explore the different ways you can interact with the robot using examples in [`ext/unitree_sdk2/example`](ext/unitree_sdk2/example/).


## Usage
You have two possibilities, either you develop a program to be run in C++ only, as in the [quick start](#quick-start) example or you create a program meant to be run in Python.

### C++ script
To add your own C++ script, make sure to add this line to [`CMakeLists.txt`](CMakeLists.txt): 
```
add_executable(<your_exec_name> <your_source_file>)
```

Then, as you did for the `unitree_sdk2`, rebuild your project:
```bash
mkdir build
cd build
cmake ..
make
```

And run your executable:
```bash
sudo ./<your_exec_name> <network_interface_name>
```

### Python script

We also needed to send commands to the robot in Python. To do so, we created bindings for our C++ code to be used directly from Python. Learn more about bindings in the official [pybind11 repo](https://github.com/pybind/pybind11).

To make your scripts compatible with `pybind11`, you simply need to include it and define a module that defines how the Python code should be generated. You can find an example in [VelocityController.cpp](src/VelocityController.cpp).

Then, add this line to [`CMakeLists.txt`](CMakeLists.txt), both names **must** match:
```
pybind11_add_module(<script_name> src/<script_name>.cpp)
```

Don't forget to rebuild the project.

You can now test your script. Inside the `build` directory, start a Python Shell session:
```bash
cd build
python3
```
```python
from VelocityController import VelocityController
controller = VelocityController("<network_interface_name>")
controller.move(0.5, 0)
controller.rotate(3.1415)
controller.stop()
```
