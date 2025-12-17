#!/usr/bin/env python3
"""Launch file to test driver with pcap replay."""

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, RegisterEventHandler, Shutdown, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # pcap_file argument must be declared before OpaqueFunction uses it
    pcap_file_arg = DeclareLaunchArgument(
        'pcap_file',
        default_value='',
        description='Absolute path to pcap file (overrides YAML if set)'
    )
    # Package paths
    pkg_share = get_package_share_directory('pingdsp_driver')

    # Robust: use only package-relative paths for configs/scripts, user-provided for data
    scripts_dir = os.path.join(pkg_share, 'scripts')

    # Add a launch argument for the paths config file
    default_paths_config = os.path.join(pkg_share, 'config', 'paths.yaml')
    paths_config_arg = DeclareLaunchArgument(
        'paths_config',
        default_value=default_paths_config,
        description='YAML file specifying network_dump_dir and bag_dir'
    )

    # Launch arguments for other parameters
    replay_speed_arg = DeclareLaunchArgument(
        'replay_speed',
        default_value='50.0',
        description='Replay speed multiplier (1.0=realtime, 10=10x)'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', '3dss_params.yaml'),
        description='Path to driver config file'
    )
    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value=os.path.join(pkg_share, 'config', 'filter_params.yaml'),
        description='Path to filter config file'
    )
    recorder_config_arg = DeclareLaunchArgument(
        'recorder_config',
        default_value=os.path.join(pkg_share, 'config', 'recorder_params.yaml'),
        description='Path to recorder config file'
    )
    enable_filter_arg = DeclareLaunchArgument(
        'enable_filter',
        default_value='true',
        description='Enable point cloud filtering'
    )
    record_pointcloud_arg = DeclareLaunchArgument(
        'record_pointcloud',
        default_value='false',
        description='Enable point cloud recording to PLY file'
    )
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Enable ROS 2 bag recording of all topics'
    )
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='',
        description='Name for the ROS 2 bag recording (without extension)'
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.expanduser('~/sonar_data'),
        description='Directory to save point cloud files'
    )

    def launch_setup(context, *args, **kwargs):
        # Get runtime values
        paths_config = LaunchConfiguration('paths_config').perform(context)
        fallback_network_dump = os.path.expanduser('~/network_dump')
        fallback_bag_dir = os.path.expanduser('~/bags')
        try:
            with open(paths_config, 'r') as f:
                paths = yaml.safe_load(f)
                network_dump_dir = paths.get('network_dump_dir', fallback_network_dump)
                bag_dir = paths.get('bag_dir', fallback_bag_dir)
        except Exception:
            network_dump_dir = fallback_network_dump
            bag_dir = fallback_bag_dir


        # Always use the YAML value for the pcap file path unless overridden by launch argument
        pcap_file_arg_val = LaunchConfiguration('pcap_file').perform(context)
        # If the user did not override, use the YAML path
        if pcap_file_arg_val and pcap_file_arg_val != '':
            pcap_file = pcap_file_arg_val
        else:
            pcap_file = os.path.join(network_dump_dir, 'asko_survey_bigrange.pcap')
        print(f"[LAUNCH DEBUG] Using pcap_file: {pcap_file}")
        bag_name_arg_val = LaunchConfiguration('bag_name').perform(context)
        bag_name = bag_name_arg_val if bag_name_arg_val else os.path.splitext(os.path.basename(pcap_file))[0]

        scripts_dir = os.path.join(pkg_share, 'scripts')
        replay_speed = LaunchConfiguration('replay_speed').perform(context)
        config_file = LaunchConfiguration('config_file').perform(context)
        filter_config = LaunchConfiguration('filter_config').perform(context)
        recorder_config = LaunchConfiguration('recorder_config').perform(context)
        enable_filter = LaunchConfiguration('enable_filter').perform(context)
        record_pointcloud = LaunchConfiguration('record_pointcloud').perform(context)
        record_bag = LaunchConfiguration('record_bag').perform(context)
        output_dir = LaunchConfiguration('output_dir').perform(context)

        replay_cmd = [
            'python3',
            os.path.join(scripts_dir, 'replay_pcap.py'),
            pcap_file,
            '--speed', replay_speed
        ]
        replay_server = ExecuteProcess(
            cmd=replay_cmd,
            cwd=scripts_dir,
            output='screen',
            name='replay_server'
        )

        driver_node = Node(
            package='pingdsp_driver',
            executable='tdss_driver',
            name='tdss_driver',
            output='screen',
            parameters=[
                config_file,
                {'sonar_host': '127.0.0.1'},
                {'sonar_port': 23848}
            ]
        )
        delayed_driver = TimerAction(
            period=1.0,
            actions=[driver_node]
        )
        filter_node = Node(
            package='pingdsp_driver',
            executable='pointcloud_filter',
            name='pointcloud_filter',
            output='screen',
            parameters=[filter_config],
            condition=IfCondition(enable_filter)
        )
        delayed_filter = TimerAction(
            period=1.5,
            actions=[filter_node]
        )
        recorder_node = Node(
            package='pingdsp_driver',
            executable='pointcloud_recorder',
            name='pointcloud_recorder',
            output='screen',
            parameters=[recorder_config],
            condition=IfCondition(record_pointcloud)
        )
        delayed_recorder = TimerAction(
            period=2.5,
            actions=[recorder_node]
        )
        bag_recorder = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', bag_name],
            output='screen',
            name='bag_recorder',
            cwd=bag_dir,
            condition=IfCondition(record_bag)
        )
        delayed_bag_recorder = TimerAction(
            period=1.0,
            actions=[bag_recorder]
        )
        shutdown_on_driver_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=driver_node,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[Shutdown(reason='Driver finished')]
                    )
                ]
            )
        )
        return [
            replay_server,
            delayed_driver,
            delayed_filter,
            delayed_recorder,
            delayed_bag_recorder,
            shutdown_on_driver_exit
        ]

    return LaunchDescription([
        paths_config_arg,
        pcap_file_arg,
        replay_speed_arg,
        config_file_arg,
        filter_config_arg,
        recorder_config_arg,
        enable_filter_arg,
        record_pointcloud_arg,
        record_bag_arg,
        bag_name_arg,
        output_dir_arg,
        OpaqueFunction(function=launch_setup)
    ])
