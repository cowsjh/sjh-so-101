import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    urdf_file_name = "so101_new_calib.urdf"
    rviz_file_name = "so101.rviz"

    urdf = os.path.join(
        get_package_share_directory('so101_description'),
        'urdf',
        urdf_file_name)

    rviz = os.path.join(
        get_package_share_directory('so101_description'),
        'rviz',
        rviz_file_name)


    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    return LaunchDescription([
        Node(
            package= 'robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz]
        )
    ])