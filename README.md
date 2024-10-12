![Apache 2.0 License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)
# Clearpath Simulator for Gazebo Harmonic

A modified [Clearpath Simulator](https://github.com/clearpathrobotics/clearpath_simulator) that uses ROS 2 Jazzy, Gazebo Harmonic and Moveit2 Jazzy packages.

## Compatibility

* Ubuntu 22.04: Requires three workspaces in order ROS 2 Jazzy --> Gazebo Harmonic --> Moveit2 Jazzy
* Ubuntu 24.04: **TODO**

## Setup instructions
* Intall these prerequisits first

```bash
sudo apt-get install libyaml-cpp-dev # needed by ros_gz_bridge
pip3 install catkin_pkg
```

* Source build yaml-cpp if you don't have it installed already: https://github.com/jbeder/yaml-cpp

```bash
cd ~/Downloads
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake ..
sudo make -j4
sudo make install -j4
```

* Build a ROS 2 Jazzy source

```bash
cd ~
git clone https://github.com/Mechazo11/clearpath_simulator_harmonic_ws.git
cd clearpath_simulator_harmonic_ws/
mkdir src
vcs import src < project.repos --recursive
rosdep install -r --from-paths src --rosdistro jazzy -i -y
source ~/ubuntu22_jazzy_ws/install/setup.bash
source ~/gazebo_ws/install/setup.bash
source ~/moveit2_jazzy_ws/install/setup.bash
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w"
```

* Before running build

```sudo apt remove ros-humble*```


* Test launch: **TODO**


```bash
CMake Error at /home/tigerwife/mppi_rose25_ws/install/rosidl_cmake/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:178 (message):
  Unable to generate service interface for 'srv/ConfigureMcu.srv'.  In order
  to generate service interfaces you must add a depend tag for 'service_msgs'
  in your package.xml.
Call Stack (most recent call first):
  CMakeLists.txt:18 (rosidl_generate_interfaces)
```

* same error for ros2_control

```bash

--- stderr: controller_manager_msgs                                                                            
CMake Error at /home/icore/mppi_rose25_ws/install/rosidl_cmake/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:178 (message):
  Unable to generate service interface for 'srv/ConfigureController.srv'.  In
  order to generate service interfaces you must add a depend tag for
  'service_msgs' in your package.xml.
Call Stack (most recent call first):
  CMakeLists.txt:32 (rosidl_generate_interfaces)

```
