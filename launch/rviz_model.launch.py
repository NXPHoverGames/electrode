from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, NotSubstitution, AndSubstitution, OrSubstitution
from launch.actions import DeclareLaunchArgument, Shutdown, LogInfo, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('model',
        description='model path'
    ),
    DeclareLaunchArgument('use_texture',
        default_value=['true'],
        description='use built in model texture'
    ),
]

def generate_launch_description():

    rviz_model = Node(
        package='electrode',
        executable='rviz_model.py',
        output='log',
        parameters=[
            {'use_texture': LaunchConfiguration('use_texture')},
            {'model': LaunchConfiguration('model')},
            ],
        on_exit=Shutdown()
    )

    return LaunchDescription(ARGUMENTS + [
        rviz_model,
    ])
