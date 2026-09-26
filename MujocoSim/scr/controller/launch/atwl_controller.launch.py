import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _load_atwl_robot_description():
    xacro_path = os.path.join(
        get_package_share_directory("controller"),
        "urdf",
        "atwl.urdf.xacro",
    )
    return xacro.process_file(xacro_path).toprettyxml(indent="  ")


def generate_launch_description():
    show_gui = LaunchConfiguration("show_gui")
    simulation_frequency = LaunchConfiguration("simulation_frequency")
    realtime_factor = LaunchConfiguration("realtime_factor")

    robot_description = {"robot_description": _load_atwl_robot_description()}
    controller_yaml = os.path.join(
        get_package_share_directory("controller"),
        "config",
        "atwl_controller.yaml",
    )
    mujoco_model_path = os.path.join(
        get_package_share_directory("atwl"),
        "model",
        "scene.xml",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controller_yaml,
            {"simulation_frequency": simulation_frequency},
            {"realtime_factor": realtime_factor},
            {"robot_model_path": mujoco_model_path},
            {"show_gui": show_gui},
        ],
        output="screen",
    )

    mujoco_sim_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mujoco_sim_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    at_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "at_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[mujoco_sim_controller_spawner],
        )
    )

    load_at_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=mujoco_sim_controller_spawner,
            on_exit=[at_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("show_gui", default_value="true"),
            DeclareLaunchArgument("simulation_frequency", default_value="500.0"),
            DeclareLaunchArgument("realtime_factor", default_value="1.0"),
            mujoco,
            load_controllers,
            load_at_controller,
        ]
    )
