from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for LC Stepper Motor Test GUI.
    
    This launches the Qt-based GUI application for testing LC stepper motors
    via RS485 communication.
    
    Note: Qt GUI applications typically work better when run directly with:
    ros2 run rs485_interface lc_stepper_motor_test_gui
    as they require an interactive display environment.
    """
    
    # Launch the GUI node
    # Note: The GUI application doesn't require ROS2 initialization,
    # but we use Node action for consistency with other launch files
    lc_stepper_motor_test_gui_node = Node(
        package='rs485_interface',
        executable='lc_stepper_motor_test_gui',
        name='lc_stepper_motor_test_gui',
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        lc_stepper_motor_test_gui_node,
    ])

