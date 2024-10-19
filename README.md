![Apache 2.0 License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)
# Clearpath Simulator for Gazebo Harmonic

A modified [Clearpath Simulator](https://github.com/clearpathrobotics/clearpath_simulator) that uses ROS 2 Jazzy, Gazebo Harmonic and Moveit2 Jazzy packages.

Clearpath uses a **single yaml** file to customize its robots. More details here https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/

## Compatibility

* Ubuntu 22.04: Requires three workspaces in order ROS 2 Jazzy --> Gazebo Harmonic --> Moveit2 Jazzy
* Ubuntu 24.04: **TODO**

## Ensure ROS 2 Humble global or source built workspace is not sourced

* **EXPERIMENTAL** If you have previously used ROS 2 Humble, you need to first ensure that the ROS 2 Humble global workspace is not sourced. Follow the steps below

```bash
echo $PATH # check /opt/ros/humble is included in the PATH variable
export PATH=$(echo $PATH | tr ':' '\n' | grep -v "/opt/ros/humble" | tr '\n' ':' | sed 's/:$//')
```


* **WARNING!!** in the event the above mehod doesn't work, the easiest thing to do is to remove ROS 2 Humble binaries from **system-level** using ```sudo apt remove ros-humble*```. Please note, if you need to use ROS 2 Humble again, you would need to reinstall the binaries or build ROS 2 humble from source.


* Intall the following prerequisits first

```bash
sudo apt update
sudo apt upgrade
sudo apt-get install libyaml-cpp-dev
pip3 install catkin_pkg
```

* Build the three workspaces shown below. Make sure they are done exactly in the sequence shown belo
  
  * Build a **ROS 2 Jazzy** source from source: https://github.com/Mechazo11/ubuntu22_jazzy_ws
  
  * Build **Gazebo Harmonic** workspace from from source: https://github.com/Mechazo11/gazebo_harmonic_ws
  
  * Build Moveit2 (Jazzy compatible) version from source: https://github.com/Mechazo11/moveit2_jazzy_ws
  
  * Build this workspace

```bash
cd ~
git clone https://github.com/Mechazo11/clearpath_simulator_harmonic_ws.git
cd clearpath_simulator_harmonic_ws/
mkdir src
vcs import src < clearpath_sim.repos --recursive
rosdep install -r --from-paths src --rosdistro jazzy -i -y
source ~/ubuntu22_jazzy_ws/install/setup.bash
source ~/gazebo_harmonic_ws/install/setup.bash
source ~/moveit2_jazzy_ws/install/setup.bash
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w"
```

* Create a ```clearpath``` directory in ```/home```. This is where all [robot yaml](https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/) files that define a certain clearpath robot build will be housed.

* Test launch: Source all underlay workspaces in sequence

```bash
source ~/ubuntu22_jazzy_ws/install/setup.bash
source ~/gazebo_harmonic_ws/install/setup.bash
source ~/moveit2_jazzy_ws/install/setup.bash
source ~/clearpath_simulator_harmonic_ws/install/setup.bash
```


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
