![BSD 3-Clause License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)

# Clearpath Simulator in Gazebo Harmonic :fire:

**Version: 0.1**

This is a modified version of [clearpath_simulator](https://github.com/clearpathrobotics/clearpath_simulator) Version 0.34 from Clearpath Robotics that uses Gazebo 8 (Harmonic) as its primary simulator. Installation, how to use guide and some helpful resources are given below. 



https://github.com/user-attachments/assets/2e02e99d-9d2f-441a-b091-cf8f8014254d



:hand: If you plan on using a Xbox One/Xbox 360 controller, check ```How to install and use Xbox One Controller``` under the **Useful Resources** section. 

:hand: All tutorials in thie repository assumes you can send **TwistStamped** commands either through a controller or from keyboard or using ```rqt_joystick```.

:heavy_exclamation_mark: Please note all the original packages from clearpath_simulator v0.3 have been modified for ROS 2 Jazzy and Gazebo Harmonic compatiblity. Hence, only my forks of those packages is known to work at this time.

## Supported Features and Compatibility

- :white_check_mark: Ubuntu 22.04 (via source build)
- :white_large_square: Ubuntu 24.04 (not tested yet)
- :white_check_mark: Compatible with Gazebo Harmonic and ROS 2 Jazzy
- :white_check_mark: Xbox One S controller support
- :white_check_mark: Automatic conversion between Twist and TwistStamped messages
- :white_check_mark: Custom names for ```robot.yaml``` configuration scripts.
- :white_large_square: Realsense2 RGB-D sensor
- :white_large_square: 2D and 3D LiDAR sensor
- :white_large_square: SLAM example to save a map
- :white_large_square: Nav2 MPPI controller example
- :white_large_square: Multi-robot example

## Supported robots

- :white_check_mark: A200 Husky

## Dependencies

This simulator depends on the following packages

- Gazebo Harmonic
- Moveit2 (Jazzy version)
- Nav2 (Jazzy version)
- Packages defined in ```clearpath_sim.repos```
- Nice to haves
  - colcon: https://colcon.readthedocs.io/en/released/user/installation.html

---

## TODOs: Next release

- :white_check_mark: Test in Ubuntu 24.04
- :white_large_square Add Realsense2 RGBD sensor support
- :white_large_square Add support for multi-robot simulation

## Call for Contribution

- ```cpr_gazebo``` contains a number of custom repository worlds from Clearpath Robotics but modifying them to be compatible with Harmonic will take some time and effort. If you are interested to helping out with modifying these works and contributing to this project in general, please don't hestiate to reaching out to [me](mechazo11.github.io).

---

## Installation (Ubuntu 24.04)
<details>

* Install ROS 2 Jazzy desktop: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

* Build this workspace

```bash
cd ~
git clone https://github.com/Mechazo11/clearpath_simulator_harmonic
mkdir ~/clearpath_simulator_harmonic_ws/
cd ~/clearpath_simulator_harmonic/
vcs import src < clearpath_sim.repos --recursive
rosdep install -r --from-paths src --rosdistro jazzy -i -y
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w"
```
</details>

## Installation (Ubuntu 22.04)
<details>
<summary>Click to expand</summary>
- If you are using Ubuntu 22.04, and had previously used ROS 2 Humble, you need to first ensure that the ROS 2 Humble global workspace is not sourced. The first step is to just comment out ```source /opt/ros/humble/setup.bash``` from ```.bashrc``` file.

- [Optional] step: Remove ```/opt/ros/humble``` from global ```PATH``` variable.

```bash
echo $PATH # check /opt/ros/humble is included in the PATH variable
export PATH=$(echo $PATH | tr ':' '\n' | grep -v "/opt/ros/humble" | tr '\n' ':' | sed 's/:$//')
```

- **WARNING!!** in the event the above doesn't work, the easiest thing to do is to remove ROS 2 Humble binaries from **system-level** using ```sudo apt remove ros-humble*```. Please note, if you need to use ROS 2 Humble again, you would need to reinstall the binaries or build ROS 2 humble from source.

### Install Prerequisits

```bash
sudo apt update
sudo apt upgrade
sudo apt-get install python3-dev python3-tk libyaml-cpp-dev joystick
pip3 install numpy catkin_pkg empy lark jinja2 typeguard pyyaml 
```

### Build and Install this workspace

Build and install the following workspaces in sequence. Will take about ~1 hour and ~35-40 GB disk space
  
- Build a **ROS 2 Jazzy** workspace: <https://github.com/Mechazo11/ubuntu22_jazzy_ws>

- Build **Gazebo Harmonic** workspace: <https://github.com/Mechazo11/gazebo_harmonic_ws>

- Build **Moveit2, Nav2** (Jazzy compatible) workspace: <https://github.com/Mechazo11/moveit2_jazzy_ws>

- Build this workspace using the following steps

```bash
cd ~
git clone https://github.com/Mechazo11/clearpath_simulator_harmonic_ws.git
cd clearpath_simulator_harmonic_ws/
vcs import src < clearpath_sim.repos --recursive
rosdep install -r --from-paths src --rosdistro jazzy -i -y
source ~/ubuntu22_jazzy_ws/install/setup.bash
source ~/gazebo_harmonic_ws/install/setup.bash
source ~/moveit2_nav2_jazzy_ws/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w"
```
</details>

---

## Usage
<details>
  <!-- <summary>Click to expand</summary> -->

  ### Ubuntu 22.04 (Source build)
  <details>
  - In a new terminal, source all workspaces in the following sequence

  ```bash
  source ~/ubuntu22_jazzy_ws/install/setup.bash
  source ~/gazebo_harmonic_ws/install/setup.bash
  source ~/moveit2_jazzy_ws/install/setup.bash
  source ~/clearpath_simulator_harmonic_ws/install/setup.bash
  ```
  </details>

  ### Ubuntu 24.04 (global workspace)

  <details>

  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/clearpath_simulator_harmonic_ws/install/setup.bash
  ```
  </details>

  ### Setup Environment variables
  <details>
  * Append location of the ```world``` file to the current value of ```GZ_SIM_RESOURCE_PATH``` env variable

  ```bash
  export GZ_VERSION=harmonic 
  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/clearpath_simulator_harmonic_ws/install/clearpath_gz/share/clearpath_gz/worlds
  ```
  </details>

  ### Update udev rules [OPTIONAL]
  <details>
  * Recreate symlink and reload udev rules. This example is for xbox but the same rule applies for PS4 / PS5 controllers

  ```bash
  sudo ln -s /dev/input/js2 /dev/input/xbox
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  ls -l /dev/input/xbox
  ```
  </details>

  ### Launch simulation
  <details>

  ```bash
  ros2 launch clearpath_gz empty_launch.py robot_config_yaml:=husky_a200_sample.yaml
  ```
  
  * Test robot's movement with a TwistStamped message

  ```bash
  ros2 topic pub /a200_0000/platform_velocity_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"
  ```

  * If the robot moves, then all controller configurations have been setup correctly, now we can use a gamepad to control the robot

  * Teleop with gamepad
    - Press and Hold ```LB``` to enable teleop
    - Press and Hold ```RB``` to enable faster movement
    - Analog stick 1 controls both linear and angular velocities

  - Launch the ```warehouse_cpr``` world that brings in a A200 Husky robot

  ```bash
  ros2 launch clearpath_gz simulation.launch.py robot_config_yaml:=husky_a200_sample.yaml world:=warehouse_cpr
  ```
</details>

---

## Useful Resources
<details>
  <summary>Click to expand</summary>

  ### Websites and materials
  <details>

  - Clearpath uses a **single yaml** file to define its robots. More details here <https://docs>. clearpathrobotics.com/docs/ros/config/yaml/overview/.
  - [Simulate](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate/)
  - [Migration from Gazebo Classic: SDF](https://gazebosim.org/api/sim/8/migrationsdf.html)
  - [ROS 2 and Gazebo Integration Best Practices](https://vimeo.com/showcase/9954564/video/767127300)
  - [Spherical Coordinates](https://gazebosim.org/api/sim/8/spherical_coordinates.html)
  - [Finding resources](https://gazebosim.org/api/sim/8/resources.html)
  - [GZ_SIM_RESOURCE_PATH](https://robotics.stackexchange.com/questions/108511/what-should-gz-sim-resource-path-be-pointing-to)
  - [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)
  - [Simulation of a 4WS Robot Using ROS2 Control and Gazebo](https://www.youtube.com/watch?v=VX53gAXafUA): This example moved a 4W drive robot using ros2_control
  - teleop_twist_joy: <https://github.com/ros2/teleop_twist_joy>
  - Teleo with a joystick: <https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/teleop/>
  - teleop_twist_joy: <https://github.com/ros2/teleop_twist_joy>
  - On using Substitutions in ROS 2 launch files: <https://daobook.github.io/ros2-docs/xin/Tutorials/Launch-Files/Using-Substitutions.html>
  - Pose publisher demo: <https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/pose_publisher.sdf>
  - Documentations on using gazebo_ros2_control: <https://control.ros.org/rolling/doc/gazebo_ros2_control/doc/index.html>
  - ROS 2 Gazebo tutorial robot simulation with `ros2_control`: <https://www.youtube.com/watch?v=PM_1Nb9u-N0>
  - An excellent example for correctly defining `ros2_control` plugin names to connect with Gazebo Harmonic: <https://www.youtube.com/watch?v=u54WAlAewMU>
  - Convert Twist to TwistStamped message: <https://github.com/joshnewans/twist_stamper>
  - Complete list of Github markdown emoji support: <https://gist.github.com/rxaviers/7360908>

  </details>


  ### Basics of Gazebo topic CLI
  <details>
  - To list all published topic ```gz topic -l```
  - To echo a gz topic: ```gz topic -e --topic  /model/a200_0000/robot/cmd_vel```
  </details>

  ### Some notes on this software stack
  <details>
  - Joystick nodes are launched from the ```clearpath_common/clearpath_control/teleop_joy.launch.py``` file
  - To find out where `ros2_control`, `gazebo` plugins and `ros_gz_bridge` elements of A200 Husky robots are defined, start looking into ```clearpath_common/clearpath_platform_description/urdf/a200```. The same is true for the other supported robots
  </details>

  ## How to install and use Xbox One Controller [OPTIONAL]
  <details>
  The following instructions are valid for a Xbox One game controller and Ubuntu 22.04 / 24/04.

  - Ensure ```dkms```, ```bluez``` and ```xpadneo``` drivers are installed. Bu default ```dkms``` and ```linux headers``` will be installed in Ubuntu 22.04. Install ```bluez```: ```sudo apt-get install bluez```

  - Install ```xpadneo```

  ```bash
  cd ~Downloads/
  git clone https://github.com/atar-axis/xpadneo.git
  cd ~xpandneo/
  sudo ./install.sh
  ```

  - Pair a Xbox controller, follow the steps shown below

  <img src="docs/gamepad_connection.png" alt="alt text" style="height:500px; width:auto; object-fit: cover;">

  - Clone ```joy_tester``` library and build it

  ```bash
  cd ~/clearpath_simulator_harmonic_ws/src
  git clone https://github.com/joshnewans/joy_tester.git
  cd ..
  colcon build --symlink-install --packages-select joy_tester
  source ./install/setup.bash
  ```

  - Then launch the ```joy_tester``` package to test if all the buttons are working properly.

  - In one terminal, run the teleop_twist_joy

  ```bash
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
  ```

  - In the other terminal run the ```test_joy```

  ```bash
  source ~/clearpath_simulator_harmonic_ws/install/setup.bash
  ros2 run joy_tester test_joy
  ```

  - If successfull you should see something like the following
  <img src="docs/joy_test.png" alt="alt text" style="height:500px; width:auto; object-fit: cover;">

  - Now identify which ```jsx``` represents the connected gamepad. First find out how many ```jsx``` device nodes are there

  ```bash
  ls /dev/input/js*
  ```

  - Test each node until you find the one that reacts with a button press. In my case it was ```js2``` node

  ```bash
  jstest /dev/input/js2
  ```

  - Now create a symbolic link between this node and ```/dev/input/xbox``` and create udev rule

    - Create symbolic links and ```udev``` rule: ```sudo ln -s /dev/input/js2 /dev/input/xbox```
    - Identify unique properties:
    
    ```bash
    udevadm info -a -n /dev/input/js2 | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{name}'
    ```.
    
    An **example** is shown below, DO NOT COPY THESE

    ```bash
      ATTRS{name}=="Xbox Wireless Controller"
      ATTRS{idProduct}=="0032"
      ATTRS{idVendor}=="8087"
      ATTRS{idProduct}=="0608"
      ATTRS{idVendor}=="05e3"
      ATTRS{idProduct}=="0002"
      ATTRS{idVendor}=="1d6b"
    ```

    - Create udev rule file: ```sudo nano /etc/udev/rules.d/99-xbox-controller.rules``` and copy these attributes (after filling them out wiht idVendor and idProduct unique to your controller)

    ```bash
    SUBSYSTEM=="input", KERNEL=="js[0-9]*", ATTRS{idVendor}=="05e3", ATTRS{idProduct}=="0002", SYMLINK+="input/xbox"
    ```
    
    Make sure to change with actual values
    - Reload Udev rules and trigger

    ```bash
      sudo udevadm control --reload-rules
      sudo udevadm trigger
    ```

    - Verify simlink: ```ls -l /dev/input/xbox``` you should see something like this
    <img src="docs/symlink.png" alt="alt text" style="height:50px; width:auto; object-fit: cover;">
  </details>
</details>

---
