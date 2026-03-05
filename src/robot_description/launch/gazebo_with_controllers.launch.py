import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    arduinobot_description_dir = get_package_share_directory("robot_description")
    world_file = os.path.join(arduinobot_description_dir, 'worlds', 'empty_bullet.sdf')

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(arduinobot_description_dir, "urdf", "robot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(arduinobot_description_dir).parent.resolve())]
    )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration("model"), ' is_ignition:=', is_ignition]),
        value_type=str
    )

    # Un solo robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file, '--physics-engine', 'gz-physics-bullet-featherstone-plugin'],
        output='screen'
    )

    # Spawn del robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "robo2"]
    )

    # Bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    # Spawners de controladores (esperan a que se spawne el robot)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"]
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"]
    )

    # Eventos para cargar controladores después del spawn
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    load_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
    ])