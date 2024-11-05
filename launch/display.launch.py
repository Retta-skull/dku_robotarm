from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지의 공유 디렉토리 경로 가져오기
    package_share_directory = get_package_share_directory('dku_robotarm')

    # URDF 파일과 RViz 설정 파일의 상대 경로 설정
    urdf_file = os.path.join(package_share_directory, 'urdf', 'dku_robotarm.urdf')
    rviz_display_config_file = os.path.join(package_share_directory, 'rviz', 'dku_robotarm.rviz')

    try:
        with open(urdf_file, 'r') as infp:
            robot_description_file = infp.read()
    except Exception as e:
        print(f"Failed to read URDF file: {e}")
        robot_description_file = ""



    ld = LaunchDescription()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': robot_description_file}
        ],
        output='screen'
    )

    joint_state_publisher = Node(
        package='dku_robotarm',
        executable='JointStatePublisher.py',
        name='JointStatePublisher',
        output='screen'
    )


    mark_publisher = Node(
        package='dku_robotarm',
        executable='MarkPublisher.py',
        name='MarkPublisher',
        output='screen'
    )

    markarray_publisher = Node(
        package='dku_robotarm',
        executable='MarkerArrayPublisher.py',
        name='MarkArrayPublisher',
        output='screen'
    )
    yolo_publisher = Node(
        package='dku_robotarm',
        executable='Camera.py',
        name='yolo_detector',
        output='screen'
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_display_config_file],
        output='screen'
    )

    # 모든 노드를 LaunchDescription에 추가
    ld.add_action(robot_state_publisher)  
    ld.add_action(joint_state_publisher)      
    ld.add_action(mark_publisher)
    ld.add_action(yolo_publisher)
    ld.add_action(markarray_publisher)
    # ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz2)

    return ld
