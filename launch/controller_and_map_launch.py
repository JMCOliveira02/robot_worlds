import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    world_dir = get_package_share_directory('robot_worlds')
    robot_description_path = os.path.join(world_dir, 'urdf', 'robot.urdf')

    robot_controller = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    webots = WebotsLauncher(
        world=os.path.join(world_dir, 'worlds', 'square_no_box.wbt')
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"yaml_filename": os.path.join(world_dir, "maps", "square_no_box.yaml")}],
        output="screen"
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"]
        }]
    )

    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",
    )

    tf_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    tf_static_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar2D"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(world_dir, "rviz", "tf_and_map.rviz")],
        output="screen"
    )


    return LaunchDescription([
        rviz,
        tf_static,
        tf_static_lidar,
        webots,
        robot_controller,
        teleop,
        map_server,
        lifecycle_manager,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])