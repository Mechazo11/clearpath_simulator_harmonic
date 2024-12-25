# DEV

Scratch book for developing new features into the simulator

- update asus xtion urdf.xacro to match the interl_realsense.urdf.xacro format.


### Realsense 2 RGBD plugin / RGBD camear plugin

* An example of AMR using RGBD camera: https://github.com/IntelligentRoboticsLabs/kobuki

* An example demo of who to use RGBD camera: https://github.com/gazebosim/gz-sim/blob/990f1c27d4f2ef69740bab433be6e8b7206d39ac/examples/worlds/sensors_demo.sdf#L452

* Example of how to launch the rgbd camera: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/launch/rgbd_camera.launch.py

* Example of turtlebot3 waffle after modified for newer gazebo: https://github.com/azeey/turtlebot3_simulations/blob/new_gazebo/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf

* Gazebo ionic migration documentations: https://gazebosim.org/docs/latest/migration_from_ignition/

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
git clone -b rolling-devel --single-branch https://github.com/ros-visualization/rqt_image_view.git
git clone -b ros2 --single-branch https://github.com/ros-drivers/openni2_camera.git
git clone -b ros2 --single-branch https://github.com/ros-drivers/nmea_msgs.git
git clone https://github.com/tilk/rtcm_msgs.git
git clone -b ros2 --single-branch https://github.com/LORD-MicroStrain/microstrain_inertial.git
git clone - b jazzy-devel --single-branch https://github.com/Mechazo11/ros2_asus_xtion.git
```

```bash
ros2 launch clearpath_gz empty_launch.py robot_config_yaml:=husky_a200_sample.yaml
ros2 launch --debug clearpath_gz empty_launch.py robot_config_yaml:=husky_a200_sample.yaml
ros2 launch clearpath_gz simulation.launch.py robot_config_yaml:=husky_a200_sample.yaml world:=warehouse_cpr

ros2 launch clearpath_gz simulation.launch.py robot_config_yaml:=husky_a200_sample.yaml world:=outdoor

```

* All model dae files found here: https://github.com/osrf/gazebo_models

### Tracking the software stack of how sensors are added into urf.xacro

clearpath_gz::robot_spwan.launch --> clearpath_generator_common::generate_description --> 
clearpath_generator_common::description::generator.DescriptionGenerator -->

**self.generate_sensors**
  | -- self.clearpath_config.sensros.get_all_sensors() <-- clearpath_config::sensors.SensorConfig.get_all_sensors() 
    | -- for each sensor in sensors
      | -- sensor_description = clearpath_generator_common::sensor.SensorDescription(sensor)
        | -- clearpath_config.sensors.types.cameras
        

### TODOs

* [x] Upgrade [ros2_asus_xiton camera](https://github.com/Mechazo11/ros2_asus_xtion.git) to utilize gazebo harmonic

* [ ] Fix camera orientation and find out why depth images are not published in ros2 graph
