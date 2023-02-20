import launch_ros.actions
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mav_params = FindPackageShare("rotors_control").find("rotors_control") + "/resources/mav_params.yaml"
    lee_controller_params = FindPackageShare("rotors_control").find("rotors_control") + "/resources/lee_controller_params.yaml"
    return LaunchDescription([
        #
        launch_ros.actions.Node(
            name="lee_position_controller_node",
            executable="lee_position_controller_node",
            package="rotors_control",
            parameters=[mav_params, lee_controller_params],
            output="screen"
        )
    ])
