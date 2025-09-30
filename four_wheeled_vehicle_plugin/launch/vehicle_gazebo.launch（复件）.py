import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare('four_wheeled_vehicle').find('four_wheeled_vehicle')
    model_path = os.path.join(pkg_path, 'models')
    model_sdf = os.path.join(model_path, 'four_wheeled_vehicle', 'model.sdf')
    world_file = os.path.join(pkg_path, 'empty.world')
    plugin_path = os.path.join(pkg_path, '../..', 'lib', 'four_wheeled_vehicle')

    os.environ['GAZEBO_MODEL_PATH'] = f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{model_path}"
    os.environ['GAZEBO_PLUGIN_PATH'] = f"{os.environ.get('GAZEBO_PLUGIN_PATH', '')}:{plugin_path}"

    return LaunchDescription([
        # 启动 Gazebo 空世界
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 加载四轮车模型
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'four_wheeled_car',
                '-file', model_sdf,
                '-x', '0.0', '-y', '0.0', '-z', '1.0'
            ],
            output='screen'
        ),

        # 启动 joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'dev': '/dev/input/js0'},
                {'deadzone': 0.05}
            ]
        ),

        # 启动 joy_to_vehicle_cmd_node
        Node(
            package='joy_to_vehicle_cmd',
            executable='joy_to_vehicle_cmd_node',
            name='joy_to_vehicle_cmd',
            output='screen',
            parameters=[
                {'speed_scale': 2.0},#速度转换比例
                {'steering_scale': 0.61}#转向转换比例
            ]
        )
    ])
