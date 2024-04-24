project(com304)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2 ext/unitree_sdk2/lib)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# Python linked scripts
add_subdirectory(ext/pybind11)
pybind11_add_module(controller src/controller.cpp)

# Pure cpp scripts
add_executable(test src/test.cpp)
