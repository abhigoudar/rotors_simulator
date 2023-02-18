from os import environ
from os import pathsep

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from scripts import GazeboRosPaths
import xacro

def setup_env():
    model = FindPackageShare("rotors_gazebo").find(package_name="rotors_gazebo") + "/models"
    plugin = FindPackageShare("rotors_gazebo").find(package_name="rotors_gazebo_plugins")

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
    }

def parse_model():
    xacro_file = FindPackageShare("rotors_description").find("rotors_description") + "/urdf/firefly_base.xacro"

    doc = xacro.process_file(xacro_file, mappings={
        "namespace": "firefly",
        "enable_mavlink_interface" : "False",
        "enable_ground_truth" : "False",
        "enable_logging" : "False",
        "log_file" : "",
        "mav_name" : "firefly",
        "wait_to_record" : "False"
    })
    desc = doc.toxml();
    return  Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'firefly'],
                        output='screen')

def generate_launch_description():

    setup_env()
    robot_model = parse_model()

    world_file = DeclareLaunchArgument(
        name = "world_file",
        default_value = PathJoinSubstitution(
            [FindPackageShare("rotors_gazebo"), "worlds/empty.world"]
        )
    )


    return LaunchDescription([
        #
        world_file,
        #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), 'launch/gzserver.launch.py']
                )
            ),
            launch_arguments={
                'world' :  LaunchConfiguration("world_file")
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), 'launch/gzclient.launch.py']
                )
            ),
        ),

        robot_model
    ])