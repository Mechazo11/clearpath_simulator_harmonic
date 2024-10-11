![BSD-3-Clause License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)
# TODO title

**TODO** a few liner description and high level pictures. pictures of result curves.

## Setup instructions

* Note ```gz_cmake_vendor, gz_math_vendor and gz_utils_vendor``` needs to be available to the underlaying/base ros 2 workspace

* Intall these prerequisits

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
git clone https://github.com/Mechazo11/mppi_rose25_ws.git
cd mppi_rose25_ws/
mkdir src
vcs import src < project.repos
rosdep install --from-paths src --ignore-src -y
source ~/ubuntu22_jazzy_ws/install/setup.bash
source ~/gazebo_ws/install/setup.bash
colcon build --symlink-install
```

* Open a terminal and execute the following commands

```bash
sudo apt update
sudo apt upgrade
rosdep update
cd ~
git clone https://github.com/Mechazo11/mppi_rose25_ws.git
cd mppi_rose25_ws/
mkdir src
vcs import src < project.repos
rosdep install -r --from-paths src --rosdistro jazzy -i -y --skip-keys "ros-humble-rosidl ros-humble-rcutils ros-humble-rcl-interfaces"

colcon build --packages-select controller_manager_msgs --packages-ignore  rosidl_cli test_msgs
colcon build --packages-select lifecycle_msgs --cmake-clean-cache
ls /home/icore/mppi_rose25_ws/install/lifecycle_msgs/share/lifecycle_msgs/msg/

colcon build --symlink-install --packages-ignore rosidl_cli test_msgs --cmake-args -DCMAKE_CXX_FLAGS="-w"

```

### Misc

* Generate a package list for moveit2

```bash
cd ~/Downloads
git clone https://github.com/moveit/moveit2.git -b main
cd moveit2/
rosinstall_generator \
  --rosdistro jazzy \
  --deps -- \
  --exclude-path ~/ubuntu22_jazzy_ws/src \
  --exclude $(cat excluded-pkgs.txt) -- \
  -- $(cat moveit2-pkgs.txt) \
  > moveit2_generated_pkgs.repos
```


See the ```Notes.md``` for some my notes.

# Project TODO

* [ ] Test and update clearpath's extra testing env to be compatible with Gazebo Harmonic (Gazebo 8). Start creating ```project.repos```

* [ ] Understand how to put the skid-steering kinematics model for a 4 wheel robot. Go through the master thesis and book given by Farid

* [ ] Start a separate repo for a ROS 2 wrapper for MPPI-Generic. Figure out if we can create a node to run the cartpole example correctly.

---

## Done

* [x] Find a suitable ackermann drive mars rover model: (ros2_rover)
* [x] Find a ROS 2 compliant skid-steering model for Clearpath Husky.
* [x] Do a one run review of the MPPI-Generic algorithm to determine how to create a wrapper for it.
* [x] Start a repo for running the project, track it directly within the workspace


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
