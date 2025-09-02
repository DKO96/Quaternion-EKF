from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ekf_imu"), "urdf", "sensor.urdf.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    pi_sensor_publisher = Node(
        package="ekf_imu",
        executable="pi_server.py",
        name="pi_server",
        output="screen"
    )

    ekf_estimator = Node(
        package="ekf_imu",
        executable="estimate",
        name="estimate",
        output="screen"
    )

    rviz_config = PathJoinSubstitution([FindPackageShare("ekf_imu"), "rviz", "sensor.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            pi_sensor_publisher,
            ekf_estimator,
            rviz
        ]
    )
