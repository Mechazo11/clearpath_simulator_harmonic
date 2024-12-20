# DEV

Scratch book for developing new features into the simulator

### Realsense 2 RGBD plugin / RGBD camear plugin

* An example of AMR using RGBD camera: https://github.com/IntelligentRoboticsLabs/kobuki

* An example demo of who to use RGBD camera: https://github.com/gazebosim/gz-sim/blob/990f1c27d4f2ef69740bab433be6e8b7206d39ac/examples/worlds/sensors_demo.sdf#L452

* Example of how to launch the rgbd camera: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/launch/rgbd_camera.launch.py


```bash
sudo apt update
sudo apt upgrade
```

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

```bash
cd ~/clearpath_simulator_harmonic/
cd src/
git clone -b ros2 --single-branch https://github.com/ros-drivers/openni2_camera.git
git clone -b ros2 --single-branch https://github.com/ros-drivers/nmea_msgs.git
git clone https://github.com/tilk/rtcm_msgs.git
git clone -b ros2 --single-branch https://github.com/LORD-MicroStrain/microstrain_inertial.git
git clone - b jazzy-devel --single-branch https://github.com/Mechazo11/ros2_asus_xtion.git

```


### TODOs

* Upgrade [ros2_asus_xiton camera](https://github.com/Mechazo11/ros2_asus_xtion.git) to utilize gazebo harmonic