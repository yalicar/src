# Create launcher for robot
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #launch robot_localization
    robot_localization = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('robot_bringup'), 'params', 'ekf.yaml')],
    )
    # launch rplidar_ros2
    rplidar_ros2 = launch_ros.actions.Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'base_laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )


    # launch slam_toolbox without odometry data and only laser data
    slam_toolbox = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'laser_frame': 'base_laser',
        }
        ],
    )
    # Launch imu_complementary_filter for imu data filtering and publishing to /imu/data topic (required by slam_toolbox) 
    imu_filter_madgwick = launch_ros.actions.Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_complementary_filter',
        output='screen',
        
    )
    # launch rviz2 for visualization slam_toolbox
    rviz2 = launch_ros.actions.Node(
        package='rviz2', # rviz2 is a package in ros2 humble
        executable='rviz2', # rviz2 is a executable in ros2 humble 
        name='rviz2', # name of the node 
        output='screen', # output of the node

    )
    # launch teleop_twist joy  xbox controller
    teleop_twist_joy = launch_ros.actions.Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'axis_linear': 1,
            'axis_angular': 0,
            'scale_linear': 0.5,
            'scale_angular': 0.5,
            'enable_button': 0,
            'enable_turbo_button': -1,
            'scale_linear_turbo': 1.0,
            'scale_angular_turbo': 1.0,
            'autorepeat_rate': 20.0,
            'joy_config': 'xbox',
        }
        ],
    )


    #return launch description
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        imu_filter_madgwick,
        robot_localization,
        rviz2,

    ])
