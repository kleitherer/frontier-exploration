#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node 
import launch_testing.actions
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", default_value="true"),
        
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("asl_tb3_sim"),
                "launch",
                "rviz.launch.py"
            ]),
            launch_arguments={
                "config": PathJoinSubstitution([
                    FindPackageShare("autonomy_repo"),
                    "rviz",
                    "default.rviz",
                ]),
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        Node(
            executable="rviz_goal_relay.py",
            package="asl_tb3_lib",
            parameters=[{"output_channel": "/cmd_nav"}],
            name="rviz_goal_relay",
        ),
        Node(
            executable="state_publisher.py",
            package="asl_tb3_lib",
            name="state_publisher",
        ),
        Node(
            package="asl_tb3_lib",
            executable="navigator",
            name="navigator",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        launch_testing.actions.TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="frontier_explorer",
                    executable="explorer_node",
                    name="frontier_explorer",
                    parameters=[{"use_sim_time": use_sim_time}],
                )
            ]
        ),
    ])