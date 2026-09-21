
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    trajectory = Node(
        package="assignment1",
        executable="trajectory",
        output="screen",)
    
    rqt_joint_trajectory_controller = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
    )

    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("asbr_description"), "/launch/ur5e_robotiq.launch.py"]
        ),
    )

    nodes_to_start = [
        launch_robot,
        rqt_joint_trajectory_controller,
        trajectory,
    ]

    return LaunchDescription( nodes_to_start )
