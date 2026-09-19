# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    caster = LaunchConfiguration('caster').perform(context)
    server_ip = LaunchConfiguration('server_ip').perform(context)
    server_port = int(LaunchConfiguration('server_port').perform(context))
    mount_point = LaunchConfiguration('mount_point').perform(context)
    user_name = LaunchConfiguration('user_name').perform(context)
    password = LaunchConfiguration('password').perform(context)
    send_position = LaunchConfiguration('send_position').perform(context).lower() == 'true'
    send_position_interval = float(LaunchConfiguration('send_position_interval').perform(context))
    auto_reconnect = LaunchConfiguration('auto_reconnect').perform(context).lower() == 'true'
    reconnect_interval = float(LaunchConfiguration('reconnect_interval').perform(context))

    # test用プリセットの適用
    if not server_ip:
        if caster == 'bizstation':
            server_ip = 'ntrip1.bizstation.jp'
            if not mount_point:
                mount_point = '3041F3CA'  # 茨城県つくば市（善意の基準局）
        elif caster == 'rtk2go':
            server_ip = 'rtk2go.com'
            if not mount_point:
                mount_point = 'TohokuUniv'

    parameters = [
        {'server_ip': server_ip},
        {'server_port': server_port},
        {'mount_point': mount_point},
        {'user_name': user_name},
        {'password': password},
        {'send_position': send_position},
        {'send_position_interval': send_position_interval},
        {'auto_reconnect': auto_reconnect},
        {'reconnect_interval': reconnect_interval},
    ]

    node = Node(
        package='tobas_ntrip_client',
        executable='ntrip_client_node',
        name='ntrip_client',
        output='screen',
        parameters=parameters,
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'caster',
            default_value='bizstation',
            description='NTRIP caster preset: "bizstation", "rtk2go", or "custom"',
        ),
        DeclareLaunchArgument(
            'server_ip',
            default_value='',
            description='NTRIP caster server IP or hostname (overrides caster preset)',
        ),
        DeclareLaunchArgument(
            'server_port',
            default_value='2101',
            description='NTRIP caster server port',
        ),
        DeclareLaunchArgument(
            'mount_point',
            default_value='',
            description='Mount point name (e.g. 3041F3CA for Tsukuba on bizstation)',
        ),
        DeclareLaunchArgument(
            'user_name',
            default_value='',
            description='Username or email for caster authentication (optional for bizstation)',
        ),
        DeclareLaunchArgument(
            'password',
            default_value='',
            description='Password for caster authentication',
        ),
        DeclareLaunchArgument(
            'send_position',
            default_value='true',
            description='Send NMEA GGA position periodically to caster',
        ),
        DeclareLaunchArgument(
            'send_position_interval',
            default_value='1.0',
            description='Interval to send NMEA GGA position in seconds',
        ),
        DeclareLaunchArgument(
            'auto_reconnect',
            default_value='true',
            description='Automatically reconnect on disconnection',
        ),
        DeclareLaunchArgument(
            'reconnect_interval',
            default_value='5.0',
            description='Reconnect check interval in seconds',
        ),
        OpaqueFunction(function=launch_setup),
    ])
