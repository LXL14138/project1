from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from q1_navigation import q1_config as C


def generate_launch_description():
    defaults = {
        'start_x': C.START_X, 'start_y': C.START_Y, 'start_yaw': C.START_YAW,
        'goal_x': C.GOAL[0], 'goal_y': C.GOAL[1],
        'goal_width': C.GOAL[2], 'goal_height': C.GOAL[3],
    }
    arguments = [DeclareLaunchArgument(name, default_value=str(value))
                 for name, value in defaults.items()]
    return LaunchDescription(arguments + [
        Node(package='q1_navigation', executable='q1_navigation', output='screen',
             arguments=[
                 '--start', LaunchConfiguration('start_x'), LaunchConfiguration('start_y'),
                 LaunchConfiguration('start_yaw'),
                 '--goal', LaunchConfiguration('goal_x'), LaunchConfiguration('goal_y'),
                 LaunchConfiguration('goal_width'), LaunchConfiguration('goal_height'),
             ],
             parameters=[{'use_sim_time': True}]),
    ])
