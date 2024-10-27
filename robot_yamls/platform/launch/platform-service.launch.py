from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_platform = FindPackageShare('clearpath_platform')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_clearpath_platform, 'launch', 'platform.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/home/icore/clearpath_simulator_harmonic_ws/robot_yamls/'
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'true'
                )
                ,
                (
                    'namespace'
                    ,
                    'a200_0000'
                )
                ,
                (
                    'use_manipulation_controllers'
                    ,
                    'true'
                )
                ,
            ]
    )

    # Nodes
    node_cmd_vel_bridge = Node(
        name='cmd_vel_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a200_0000',
        output='screen',
        arguments=
            [
                'a200_0000/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist'
                ,
                '/model/a200_0000/robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
                ,
            ]
        ,
        remappings=
            [
                (
                    'a200_0000/cmd_vel'
                    ,
                    'cmd_vel'
                )
                ,
                (
                    '/model/a200_0000/robot/cmd_vel'
                    ,
                    'platform/cmd_vel_unstamped'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_odom_base_tf_bridge = Node(
        name='odom_base_tf_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a200_0000',
        output='screen',
        arguments=
            [
                '/model/a200_0000/robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/model/a200_0000/robot/tf'
                    ,
                    'tf'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_platform)
    ld.add_action(node_cmd_vel_bridge)
    ld.add_action(node_odom_base_tf_bridge)
    return ld
