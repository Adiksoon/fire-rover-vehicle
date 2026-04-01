from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Automatyczne uruchomienie interfejsu graficznego RViz2
        ExecuteProcess(
            cmd=['rviz2'],
            name='rviz2',
            output='screen'
        ),
        
        # 2. Równoległe wypuszczenie skryptu rzutującego model "Duchów Piłek" na mapę
        ExecuteProcess(
            cmd=['/home/zygar/dev/fire-rover-vehicle/show_balls_rviz.py'],
            name='gazebo_balls_rviz_publisher',
            output='screen'
        )
    ])
