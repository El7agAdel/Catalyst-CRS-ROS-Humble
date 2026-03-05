import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def load_file(package_name: str, relative_path: str) -> str:
    abs_path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(abs_path, "r") as f:
        return f.read()

def generate_launch_description():
    # ---- Args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    run_streamer = LaunchConfiguration('run_streamer', default='true')
    world = LaunchConfiguration('world')


    camera_urdf = os.path.join(get_package_share_directory("Catalyst"), "urdf/camera.sdf")
    view_camera = LaunchConfiguration('view_camera', default='false')
    camera_topic = LaunchConfiguration('camera_topic', default='/cam/rgb/image_raw')


    # ---- URDF & controllers
    urdf_content = load_file("Catalyst", "urdf/Catalyst.urdf.xml")
    controllers_file = os.path.join(get_package_share_directory("Catalyst"), "config/Catalyst_controller.yaml")

    default_world = os.path.join(get_package_share_directory("Catalyst"), "worlds", "Catalyst_empty_world.sdf")

    # ---- Gazebo Fortress (ros_gz_sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    # ---- Publish /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"use_sim_time": use_sim_time, "robot_description": urdf_content}],
        output='screen'
    )

    # ---- Spawn robot (from /robot_description)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
        '-name', 'catalyst',
        '-x', '0.0',     # meters
        '-y', '0.0',
        '-z', '0.0',
        '-R', '0.0',     # roll (rad)
        '-P', '0.0',     # pitch
        '-Y', '0.0',    # yaw
        '-topic', 'robot_description'
                    ],
        output='screen'
    )

    spawn_camera = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'static_camera',
        '-x', '1.0', '-y', '0.0', '-z', '0.3',
        '-R', '0.0', '-P', '0.0', '-Y', '3.14159',
        '-file', camera_urdf
    ],
    output='screen'
    )
    
    camera_viewer = Node(
        package='IRB120',
        executable='camera_viewer',
        name='camera_viewer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic': camera_topic,
            'window_name': 'IRB120 Camera'
        }],
        output='screen',
        condition=IfCondition(view_camera)
    )

    camera_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/cam/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/cam/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/cam/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/cam/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/cam/stereo/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/cam/stereo/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/cam/stereo/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/cam/stereo/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ],
    output='screen'
    )


    # ---- Controllers
    arm_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file
        ],
        output='screen'
    )

    # ---- Optional: bridge /clock from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ---- Periodic trajectory streamer (continuous motion)
    trajectory_publisher = Node(
        package='Catalyst',
        executable='send_trajectory',
        name='send_trajectory',
        output='screen'
    )

    # Delay the publish a bit so controllers are ACTIVE
    delayed_streamer = TimerAction(
        period=3.0,
        actions=[trajectory_publisher],
        condition=IfCondition(run_streamer)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use simulation (Gazebo) clock if true"
        ),
        DeclareLaunchArgument(
            "run_streamer", default_value="true",
            description="Run the trajectory publisher after controllers are up"
        ),
        DeclareLaunchArgument(
            "world", default_value=default_world,
            description="Absolute path to Gazebo world SDF file"
        ),
        DeclareLaunchArgument(
            "view_camera", default_value="true",
            description="Start the OpenCV camera viewer node"
        ),
        DeclareLaunchArgument(
            "camera_topic", default_value="/cam/rgb/image_raw",
            description="Image topic to display"
        ),

        gazebo_launch,
        robot_state_publisher,
        spawn,
        arm_broadcaster_spawner,
        arm_controller_spawner,
        clock_bridge,
        delayed_streamer,
        spawn_camera,
        camera_bridge,
        # camera_viewer
    ])
