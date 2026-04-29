from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    truth_odom_topic = LaunchConfiguration("truth_odom_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "truth_odom_topic",
            default_value="/sim/drone_0/truth_odom",
            description="Truth odometry topic used by gnss_sim_node.",
        ),
        Node(
            package="gnss_sim",
            executable="gnss_sim_node",
            name="gnss_sim_node",
            output="screen",
            parameters=[{
                "truth_odom_topic": truth_odom_topic,
            }],
        ),
    ])
