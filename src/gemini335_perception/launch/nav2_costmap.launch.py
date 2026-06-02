import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    pkg_dir = get_package_share_directory('gemini335_perception')
    costmap_config = os.path.join(pkg_dir, 'config', 'costmap_local.yaml')

    costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        namespace='costmap',
        output='screen',
        parameters=[costmap_config]
    )

    # 当节点进程启动后，立刻触发 Configure
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=costmap_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == costmap_node,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    # 当节点进入 INACTIVE 状态 (Configure 完成) 后，立刻触发 Activate
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=costmap_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == costmap_node,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        costmap_node,
        configure_event,
        activate_event
    ])
