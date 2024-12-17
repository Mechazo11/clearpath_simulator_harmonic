# Changelogs

### Version 0.1

* ```warehouse.sdf``` is now ```warehouse_cpr.sdf``` and is compatible with Gazebo Harmonic.
* Uses my fork of `clearpath_simulation`: <https://github.com/Mechazo11/clearpath_simulator_harmonic>
* `clearpath_control`: Modified `teleop_base.launch.py` to call my Twist to Twist Stamped message converter node. It appears most `ros2_controllers` now requires `TwistStamped` messages to work. 
* ```a200/control.yaml``` has been modified to use the newer style of defining ```DiffDrive``` controller.
* `clearpath_gz`: A new cpp file was added that take ```Twsit``` message that comes out from the ```twist_mux``` server, converts it to ```TwistStamped``` message and then transfers it to ```platform_velocity_controller/cmd_vel``` topic.
* `clearpath_config`: This package in particular has significant changes, one major being addition of the Xbox One/360 controller support
* `clearpath_common`: A number of `.py` files that contained a contradictory licensing statement as discussed here Issue [#124](https://github.com/clearpathrobotics/clearpath_common/issues/124#issuecomment-2520703993) has been removed.

---

### Version 0.2

* Added `CONTRIBUTION.md` file
* **TODO** Add support for Realsense2 RGBD camera
* **TODO** add support for multi-robot simulation

