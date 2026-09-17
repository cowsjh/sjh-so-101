import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('so101_description')

    urdf = os.path.join(pkg, 'urdf', 'so101_new_calib.urdf')
    controllers = os.path.join(pkg, 'config', 'controllers.yaml')
    rviz = os.path.join(pkg, 'rviz', 'so101.rviz')

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str)
    }

    # controller_manager: URDF의 <ros2_control> 읽어 SO101SystemHardware 로드,
    # controllers.yaml 대로 jsb/arm_controller 인스턴스화. read()/write() 루프 주인.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        output='screen',
    )

    # 서보 현재값(hw_positions_) → /joint_states 로 퍼블리시
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # jtc: 목표 궤적 받아 hw_commands_ 에 씀 (팔 5축)
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )

    # 그리퍼 1축 (여닫이)
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
    )

    # /joint_states → TF (RViz 시각화용). GUI slider 없음 — 실물이 소스.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz],
    )

    return LaunchDescription([
        control_node,
        rsp_node,
        jsb_spawner,
        arm_spawner,
        gripper_spawner,
        rviz_node,
    ])
