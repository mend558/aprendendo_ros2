from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('pacote_de_exemplos'), '/launch/simulation.launch.py']),
           launch_arguments={
                'world_path': [get_package_share_directory('pacote_de_exemplos'), '/simulation/worlds/simple_room_with_fixed_boxes.world'],
            }.items(),
    )

    load_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('pacote_de_exemplos'),'/launch/load.launch.py']),
        launch_arguments={
            'rvizconfig': [get_package_share_directory('pacote_de_exemplos'), '/config/rviz/navigation.rviz'],
        }.items()        
    )

    ld = LaunchDescription()

    ld.add_action(simulation)
    ld.add_action(load_robot)
    
    return ld
