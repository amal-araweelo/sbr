
import launch
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnExecutionComplete
from launch.events.process import ProcessIO

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("assignment2"), "rviz", "view_robot.rviz"]
    )

    rqt_joint_trajectory_controller = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
    )

    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("asbr_description"), "/launch/ur5e_robotiq.launch.py"]),
        launch_arguments={
            "launch_rviz": "false",
            "use_mock_hardware": "false",
            "sim_gazebo": "true",
        }.items(),
    )

    artag_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("assignment2"), "urdf/artag.urdf.xacro"]),
        ]
    )
    artag_description = {"robot_description": artag_description_content}

    ignition_spawn_artag = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            artag_description_content,
            "-name",
            "artag",
        ],
    )

    artag_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        name="artag_state_publisher",
        namespace="artag",
        parameters=[{"use_sim_time": True}, artag_description],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen',
    )

    aruco = Node(
        package="aruco_ros",
        executable="single",
        name="aruco",
        parameters=[{"marker_id": 1,
                     "marker_size": 0.1,
                     "marker_frame": "marker",
                     "camera_frame": "camera_link",
                     "image_is_rectified": True}],
        remappings=[("/image", "/image_raw")]
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    rqt_joint_trajectory_controller = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
    )

    rqt_event = RegisterEventHandler(
        OnExecutionComplete(
            target_action=ignition_spawn_artag,
            on_completion = [
                LogInfo(msg="Starting RQT joint trajectory controller"),
                rqt_joint_trajectory_controller,
                #TimerAction(
                #    period=5.0,
                #    actions=[rqt_joint_trajectory_controller],
                #),
            ],
        ),
    )
    
    nodes_to_start = [
        launch_robot,
        rviz,
        ignition_spawn_artag,
        artag_state_publisher_node,
        bridge,
        aruco,
        rqt_event,
        rqt_joint_trajectory_controller
    ]

    return LaunchDescription( nodes_to_start )
