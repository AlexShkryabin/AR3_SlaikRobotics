from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Просто запускаем наш Python-узел. 
    # Все параметры MoveIt берутся из уже запущенного move_group.
    teleop_node = Node(
        package="ar4_teleop_gui",
        executable="teleop_gui",
        name="teleop_gui_node",
        output="screen"
    )
    
    return LaunchDescription([teleop_node])
