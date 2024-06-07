# COM-304 Unitree Go2 Project

## Folder structure

### `com304`

All our files should lie in there. The `pybind11` submodule is included. It is used to wrap the `C++` code we create in `Python` to call it from `habitat`.

### `unitree_sdk2`

It is a submodule containing the official SDK for the Unitree Go2 along with examples.

## How to run

### Direct `C++` control

To run the `C++` scripts directly you need to build the files (either from `com304` or `unitree_sdk2`):
```console
mkdir build
cd build
cmake ..
make
```

Then, you can run your executable: `sudo ./<exec_name>`. For example `sudo ./test <network interface name>`.

To run an example from the sdk here are all the commands:
```console
cd unitree_sdk2

# Do this routine only once, the first time you build the project
sudo ./install.sh
mkdir build
cd build

# Do this routine everytime you modify something to a script
cmake ..
make

# Run the script of your choice
sudo ./sportmode_test <network_interface_name>
```

### Indirect `Python` control

As `habitat` is in `Python`, we need a wrapper to call the SDK from `Python`. To do so, we use a library called `pybind11`.

`controller.cpp` contains a minimalist script to demonstrate this wrapper capability. To run it from the command line interpreter do the following:

```console
cd com304/build
cmake ..
make

sudo python3
>>> import controller
>>> cont = controller.Controller("<network_interface_name>")
>>> cont.move(0.5, 0) # Moves 0.5 meters forward
```

## Add your scripts

### Direct `C++` scripts
Add this line to the `CMakeLists.txt` of your project: `add_executable(<exec_name> <filename.cpp>)`

### Wrapped `C++` scripts
Add this line to the `CMakeLists.txt` of your project: `pybind11_add_module(<module_name> <module_name>.cpp)`.
