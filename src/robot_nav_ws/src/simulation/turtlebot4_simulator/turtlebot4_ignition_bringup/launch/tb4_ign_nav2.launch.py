#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # -------- Launch-Argumente --------
    ns_arg   = DeclareLaunchArgument('namespace', default_value='',
                                     description='Optional ROS namespace')
    map_arg  = DeclareLaunchArgument('map', default_value='map_bigger.yaml',
                                     description='Pfad zur Map YAML (absolut oder relativ zum CWD)')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='RViz für den Simulator starten (true/false)')

    namespace = LaunchConfiguration('namespace')
    map_yaml  = LaunchConfiguration('map')
    rviz_flag = LaunchConfiguration('rviz')

    # -------- Pfade zu den bestehenden Launchfiles --------
    tb4_ignition_launch = PathJoinSubstitution([
        FindPackageShare('turtlebot4_ignition_bringup'),
        'launch',
        'turtlebot4_ignition.launch.py'
    ])

    tb4_localization_launch = PathJoinSubstitution([
        FindPackageShare('turtlebot4_navigation'),
        'launch',
        'localization.launch.py'
    ])

    tb4_nav2_launch = PathJoinSubstitution([
        FindPackageShare('turtlebot4_navigation'),
        'launch',
        'nav2.launch.py'
    ])

    # -------- 1) Simulator starten --------
    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb4_ignition_launch),
        launch_arguments={
            'rviz': rviz_flag,
            'namespace': namespace
        }.items()
    )

    # -------- 2) Localization nach kurzer Verzögerung --------
    localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb4_localization_launch),
        launch_arguments={
            'map': map_yaml,
            'namespace': namespace
        }.items()
    )
    localization_after_sim = TimerAction(
        period=5.0,  # Sekunden; ggf. anpassen
        actions=[localization_include]
    )

    # -------- 3) Nav2 nach weiterer Verzögerung --------
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb4_nav2_launch),
        launch_arguments={
            'namespace': namespace
        }.items()
    )
    nav2_after_localization = TimerAction(
        period=10.0,  # Sekunden ab Launch-Start; startet effektiv nach Localization
        actions=[nav2_include]
    )

    return LaunchDescription([
        ns_arg, map_arg, rviz_arg,
        sim_include,
        localization_after_sim,
        nav2_after_localization
    ])
