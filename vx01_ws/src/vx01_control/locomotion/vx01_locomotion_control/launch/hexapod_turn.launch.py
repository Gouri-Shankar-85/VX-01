#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import launch_ros.events.lifecycle
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('vx01_locomotion_control'),
        'config', 'hexapod_locomotion.yaml'
    )

    node = LifecycleNode(
        package='vx01_locomotion_control',
        executable='hexapod_turn_node',
        name='hexapod_turn_node',
        namespace='',
        parameters=[config, {'use_sim_time': False}],
        output='screen',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name(
                node_name='hexapod_turn_node'
            ),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name(
                            node_name='hexapod_turn_node'
                        ),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([node, configure_event, activate_handler])