# gazebo.launch.py

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    arduinobot_description_dir = get_package_share_directory("robot_description")


    # World con motor Bullet
    world_file = os.path.join(arduinobot_description_dir, 'worlds', 'empty_bullet.sdf')


    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
            arduinobot_description_dir, "urdf", "robot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(arduinobot_description_dir).parent.resolve())
        ]
    )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model"), ' is_ignition:=', is_ignition]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                    "use_sim_time": True}]
    )

    # Lanzar Gazebo directamente con gz sim y especificar Bullet
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file, '--physics-engine', 'gz-physics-bullet-featherstone-plugin'],
        output='screen'
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "robo2",
                   "-x", "0.0",      # Coordenada X
                   "-y", "0.0",     # Coordenada Y
                   "-z", "0.8",      # Coordenada Z
                  ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
