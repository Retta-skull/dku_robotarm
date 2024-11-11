from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    angle_setter = Node(
        package='dku_robotarm',
        executable='AngleSetter.py',
        name='AngleSetter',
        output='screen'
    )

    marker_publisher = Node(
        package='dku_robotarm',
        executable='MarkPublisher.py',
        name='MarkPublisher',
        output='screen'
    )

    robot_arm_controller = Node(
        package='dku_robotarm',
        executable='ApplicationRobotarm.py',
        name='RobotarmController',
        output='screen'
    )

    # 모든 노드를 LaunchDescription에 추가
    ld.add_action(angle_setter)
    ld.add_action(marker_publisher)
    # ld.add_action(robot_arm_controller)
    return ld
