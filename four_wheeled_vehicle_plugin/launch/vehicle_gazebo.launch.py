import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包路径
    pkg_path = FindPackageShare('four_wheeled_vehicle').find('four_wheeled_vehicle')
    model_path = os.path.join(pkg_path, 'models')
    model_sdf = os.path.join(model_path, 'four_wheeled_vehicle', 'model.sdf')
    # URDF 文件路径
    urdf_file = os.path.join(pkg_path, 'models', 'four_wheeled_vehicle', 'four_wheeled_vehicle.urdf')
    # SDF 或世界文件（这里假设空世界）
    world_file = os.path.join(pkg_path, 'empty.world')

    # Gazebo 插件路径（如果有自定义插件）
    plugin_path = os.path.join(pkg_path, '../..', 'lib', 'four_wheeled_vehicle')
    os.environ['GAZEBO_MODEL_PATH'] = f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{os.path.join(pkg_path, 'models')}"
    os.environ['GAZEBO_PLUGIN_PATH'] = f"{os.environ.get('GAZEBO_PLUGIN_PATH', '')}:{plugin_path}"

    # RViz 配置文件
    rviz_path = os.path.join(os.path.expanduser('~/Ultra_ws/src/four_wheeled_vehicle_plugin'))
    rviz_config_file = os.path.join(rviz_path, 'rviz', 'my.rviz')

    return LaunchDescription([
        # 启动 Gazebo 空世界
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 加载四轮车模型到 Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'four_wheeled_car',
                '-file', model_sdf,
                '-x', '0.0', '-y', '0.0', '-z', '0.5'
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
                {'speed_scale': 2.0},
                {'steering_scale': 0.61}
            ]
        ),

        # robot_state_publisher，用 URDF 发布 TF 树
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}],
            remappings=[('/joint_states', '/vehicle/joint_states')]
        ),

        # 启动 RViz2 并加载自定义配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
