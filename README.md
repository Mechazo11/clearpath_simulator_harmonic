![BSD-3-Clause License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)
# TODO title

**TODO** a few liner description and high level pictures. pictures of result curves.

## Setup instructions

* Open a terminal and execute the following commands

```bash
export ROS_DISTRO="humble"
rosdep update
cd ~
git clone https://github.com/Mechazo11/mppi_rose25_ws.git
cd mppi_rose25_ws/
vcs import src < project.repos
rosdep install -r --from-paths src -i --simulate

rosdep install -r --from-paths src -i -y --skip-keys "ros-humble-ign-ros2-control ros-humble-ros-gz libignition-gazebo6-dev"
colcon build --symlink-install
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