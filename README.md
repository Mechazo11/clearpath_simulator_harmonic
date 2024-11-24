![BSD 3-Clause License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)

# Clearpath Simulator for Gazebo Harmonic

A modified [Clearpath Simulator](https://github.com/clearpathrobotics/clearpath_simulator) that uses ROS 2 Jazzy, Gazebo Harmonic and Moveit2 Jazzy packages. Clearpath uses a **single yaml** file to customize its robots. More details here https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/. I have opted to use the BSD-3 license, the same as that of ```jazzy``` branch from [clearpath_simulator](https://github.com/clearpathrobotics/clearpath_simulator/tree/jazzy)

## Compatibility

* Ubuntu 22.04: Requires three workspaces in order ROS 2 Jazzy --> Gazebo Harmonic --> Moveit2 Jazzy
* Ubuntu 24.04: **TODO**

## Major Changes

* ```warehouse.sdf``` is updated to be compatible with Gazebo Harmonic

* In ROS 1 Clearpath Robotics had ```cpr_gazebo``` which contained a good number of indoor and outdoor worlds. However that simulator is no longer maintained [porting](https://github.com/Mechazo11/cpr_gazebo_ros2) them from ROS 1 would require significant time and effort. In this package, I have updated a few of the world files for use

* List of packages that were modified / needs to use my fork
   * clearpath_simulation: https://github.com/Mechazo11/clearpath_simulator_harmonic 
   * clearpath_generator_common: **TODO**
   * realsense2_description
   * realsense_gazebo_plugin

* In a number of packages like the ```clearpath_harmonic```, ```clearpath_generator_common```, the last line of the file's BSD license had the following statement 

```text
Redistribution and use in source and binary forms, with or without
modification, is not permitted without the express permission
of Clearpath Robotics.
```

* In my opinion, I find this line to be in contradiction with the BSD-3 license that was used with the simulator. The ```.py``` files where I have made the modifications to accept custom yaml file names, I have removed this last line. An issue discussing this matter with Clearpath robotics can be found here: [Issue #]()


## Useful Resources

* [Simulate](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate/)
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
sudo apt-get install python3-dev python3-tk libyaml-cpp-dev
pip3 install numpy catkin_pkg empy lark
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


* Verify installation: In a new terminal, source all workspaces in the following sequence

```bash
source source~/ubuntu22_jazzy_ws/install/setup.bash
source ~/gazebo_harmonic_ws/install/setup.bash
source ~/moveit2_jazzy_ws/install/setup.bash
source ~/clearpath_simulator_harmonic_ws/install/setup.bash
```

* Append location of the ```world``` file to the current value of ```GZ_SIM_RESOURCE_PATH``` env variable

```bash
export GZ_VERSION=harmonic 
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/clearpath_simulator_harmonic_ws/install/clearpath_gz/share/clearpath_gz/worlds
```

* Simulate a A200 Husky on a blank world, you should see the following

```bash
ros2 launch clearpath_gz empty_launch.py robot_config_yaml:=husky_a200_sample.yaml
```

## Install and verify a gamepad [OPTIONAL]

The following instructions are valid for a Xbox One game controller. For PS4/PS5 or wired controllers, please look for them online.

* Ensure ```dkms```, ```bluez``` and ```xpadneo``` drivers are installed. Bu default ```dkms``` and ```linux headers``` will be installed in Ubuntu 22.04. Install ```bluez```: ```sudo apt-get install bluez```

* Install ```xpadneo```

```bash
cd ~Downloads/
git clone https://github.com/atar-axis/xpadneo.git
cd ~xpandneo/
sudo ./install.sh
```

* Pair a Xbox controller, follow the steps shown below

<img src="docs/gamepad_connection.png" alt="alt text" style="height:500px; width:auto; object-fit: cover;">

* Clone ```joy_tester``` library and build it

```bash
cd ~/clearpath_simulator_harmonic_ws/src
git clone https://github.com/joshnewans/joy_tester.git
cd ..
colcon build --packages-select joy_tester
source ./install/setup.bash
```

* Then launch the ```joy_tester``` package to test if all the buttons are working properly.

  * In one terminal, run the teleop_twist_joy
  ```bash
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
  ```

  * In the other terminal run the ```test_joy```
  ```bash
  source ~/clearpath_simulator_harmonic_ws/install/setup.bash
  ros2 run joy_tester test_joy
  ```

  * If successfull you should see something like the following
  <img src="docs/joy_test.png" alt="alt text" style="height:500px; width:auto; object-fit: cover;">


**TODO** based on this https://github.com/ros2/teleop_twist_joy/blob/rolling/launch/teleop-launch.py, add the teleop_twist_joy

* Launch the ```warehouse_cpr``` world that brings in a A200 Husky robot

```bash
ros2 launch clearpath_gz simulation.launch.py robot_config_yaml:=husky_a200_sample.yaml
```

---

### TODO

* [x] Fix the ```warehouse``` world, ensure husky robot simulates correctly

* [ ] Write instructions and test driving robot around with a gamepad / rqt_joystick

* [ ] Add a command that creates a folder uniquely named as the custom yaml folder's name
instead of ***robot_yamls***. 

### Useful resources

* teleop_twist_joy: https://github.com/ros2/teleop_twist_joy
* Teleo with a joystick: https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/teleop/
* teleop_twist_joy: https://github.com/ros2/teleop_twist_joy
* On using Substitutions in ROS 2 launch files: https://daobook.github.io/ros2-docs/xin/Tutorials/Launch-Files/Using-Substitutions.html