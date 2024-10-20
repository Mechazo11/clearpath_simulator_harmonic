![Apache 2.0 License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

# Clearpath Simulator for Gazebo Harmonic

A modified [Clearpath Simulator](https://github.com/clearpathrobotics/clearpath_simulator) that uses ROS 2 Jazzy, Gazebo Harmonic and Moveit2 Jazzy packages. Clearpath uses a **single yaml** file to customize its robots. More details here https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/

## Compatibility

* Ubuntu 22.04: Requires three workspaces in order ROS 2 Jazzy --> Gazebo Harmonic --> Moveit2 Jazzy
* Ubuntu 24.04: **TODO**

## Major Changes

* ```warehouse.sdf``` is updated to be compatible with Gazebo Harmonic

* In ROS 1 Clearpath Robotics had ```cpr_gazebo``` which contained a good number of indoor and outdoor worlds. However that simulator is no longer maintained [porting](https://github.com/Mechazo11/cpr_gazebo_ros2) them from ROS 1 would require significant time and effort. In this package, I have updated a few of the world files for use

## Useful Resources

* [Migration from Gazebo Classic: SDF](https://gazebosim.org/api/sim/8/migrationsdf.html)
* [ROS 2 and Gazebo Integration Best Practices](https://vimeo.com/showcase/9954564/video/767127300)
* [Spherical Coordinates](https://gazebosim.org/api/sim/8/spherical_coordinates.html)
* [Finding resources](https://gazebosim.org/api/sim/8/resources.html)
* [GZ_SIM_RESOURCE_PATH](https://robotics.stackexchange.com/questions/108511/what-should-gz-sim-resource-path-be-pointing-to)


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
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w"
```


* Test launch: Source all underlay workspaces in sequence

```bash
source ~/ubuntu22_jazzy_ws/install/setup.bash
source ~/gazebo_harmonic_ws/install/setup.bash
source ~/moveit2_jazzy_ws/install/setup.bash
source ~/clearpath_simulator_harmonic_ws/install/setup.bash
```

* Append location of the ```world``` file to the current value of ```GZ_SIM_RESOURCE_PATH``` env variable

```bash
export GZ_VERSION=harmonic 
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/clearpath_simulator_harmonic_ws/install/clearpath_gz/share/clearpath_gz/worlds
```

* Simulate a A200 Husky on a blank world ```ros2 launch clearpath_gz empty_launch.py robot_config_yaml:=husky_a200_sample.yaml```



```bash
ros2 launch clearpath_gz simulation.launch.py robot_config_yaml:=husky_a200_sample.yaml
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
