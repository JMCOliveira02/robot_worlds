import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    # Get paths from both packages
    loc_pkg = get_package_share_directory('robot_localization_package')
    worlds_pkg = get_package_share_directory('robot_worlds')

    # Paths for robot and map
    robot_description_path = os.path.join(worlds_pkg, 'urdf', 'robot.urdf')
    world_path = os.path.join(worlds_pkg, 'worlds', 'square_two_boxes.wbt')
    map_yaml_path = os.path.join(worlds_pkg, 'maps', 'square_two_boxes.yaml')
    rviz_config = os.path.join(worlds_pkg, 'rviz', 'corners_orientation.rviz')

    # Webots simulation + robot controller
    webots = WebotsLauncher(world=world_path)

    robot_controller = WebotsController(
        robot_name='robot',
        parameters=[{'robot_description': robot_description_path}]
    )

    # Launch particle filter node from localization package
    particle_filter = Node(
        package="robot_localization_package",
        executable="particle_filter",
        name="particle_filter",
        output="screen"
    )

    # Launch fake detector from robot_worlds
    fake_detector = Node(
        package="robot_worlds",
        executable="fake_detector",
        name="fake_detector",
        output="screen"
    )

    # Map server and lifecycle manager
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"yaml_filename": map_yaml_path}],
        output="screen"
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
        output="screen"
    )

    # TF static transforms
    tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    tf_base_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar2D"]
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    # Teleop (optional)
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --"
    )

    return LaunchDescription([
        #rviz,
        webots,
        robot_controller,
        fake_detector,
        #particle_filter,
        tf_map_to_odom,
        tf_base_to_lidar,
        map_server,
        lifecycle_manager,
        teleop,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
    ])
