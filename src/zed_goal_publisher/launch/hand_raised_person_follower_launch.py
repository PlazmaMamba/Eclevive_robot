#!/usr/bin/env python3

"""
Hand Raised Person Follower Launch File

This launch file starts the node that follows a person with raised hand.

Usage:
  ros2 launch zed_goal_publisher hand_raised_person_follower_launch.py

Operation:
  1. Recognize multiple people from ZED2i skeleton detection
  2. Detect person with hand raised above shoulder
  3. Record the detected person's tracking ID (label_id)
  4. Continuously follow only that person (maintaining distance and angle)
  5. Publish TF frame (target_person) for the tracking target

Verification:
  # Check tracking status
  ros2 topic echo /hand_follower/status

  # Check target ID
  ros2 topic echo /hand_follower/target_id

  # Check velocity command
  ros2 topic echo /cmd_vel

  # Check TF tree
  ros2 run tf2_tools view_frames
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    skeleton_topic_arg = DeclareLaunchArgument(
        'skeleton_topic',
        default_value='zed/zed_node/body_trk/skeletons',
        description='ZED skeleton detection topic name'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Velocity command topic name'
    )

    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.5',
        description='Target following distance [m]'
    )

    distance_tolerance_arg = DeclareLaunchArgument(
        'distance_tolerance',
        default_value='0.4',
        description='Distance tolerance error [m]'
    )

    angle_tolerance_arg = DeclareLaunchArgument(
        'angle_tolerance',
        default_value='0.1',
        description='Angle tolerance error [rad]'
    )

    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.50',
        description='Maximum linear speed [m/s]'
    )

    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.2',
        description='Maximum angular speed [rad/s]'
    )

    hand_raise_threshold_arg = DeclareLaunchArgument(
        'hand_raise_threshold',
        default_value='0.3',
        description='Hand raise detection threshold [m] (shoulder-hand height difference)'
    )

    tracking_timeout_arg = DeclareLaunchArgument(
        'tracking_timeout',
        default_value='4.0',
        description='Tracking timeout [s]'
    )

    # Node definition
    hand_raised_person_follower_node = Node(
        package='zed_goal_publisher',
        executable='hand_raised_person_follower',
        name='hand_raised_person_follower',
        output='screen',
        parameters=[{
            'skeleton_topic': LaunchConfiguration('skeleton_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'target_distance': LaunchConfiguration('target_distance'),
            'distance_tolerance': LaunchConfiguration('distance_tolerance'),
            'angle_tolerance': LaunchConfiguration('angle_tolerance'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'hand_raise_threshold': LaunchConfiguration('hand_raise_threshold'),
            'tracking_timeout': LaunchConfiguration('tracking_timeout'),
        }],
        remappings=[
            # Add remappings as needed
        ]
    )

    return LaunchDescription([
        # Launch arguments
        skeleton_topic_arg,
        cmd_vel_topic_arg,
        target_distance_arg,
        distance_tolerance_arg,
        angle_tolerance_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        hand_raise_threshold_arg,
        tracking_timeout_arg,

        # Node
        hand_raised_person_follower_node,
    ])
