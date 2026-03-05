# demo.launch.py (ROS 2 Humble)

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_file(package_name: str, relative_path: str) -> str:
    """Read a file from a package's share directory and return its contents."""
    abs_path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(abs_path, "r") as f:
        return f.read()


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Adjust the path below to match your package structure if needed
    # e.g., 'urdf/Catalyst.urdf.xml' if it lives under a 'urdf' folder.
    urdf_path = PathJoinSubstitution([
        FindPackageShare("Catalyst"),
        "urdf",
        "Catalyst.urdf.xml",
    ])

    # Resolve the substitution to an actual path at launch generation time
    # so we can read the file and pass the string to robot_description.
    # (launch resolves Substitutions lazily; to read now, we build the path manually)
    urdf_text = load_file("Catalyst", "urdf/Catalyst.urdf.xml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": urdf_text,   # pass the XML contents (string), not a path
        }],
    )

    joint_state_source = Node(
        package="Catalyst",
        executable="state_publisher",
        name="state_publisher",
        output="screen",
    )


      # --- RViz2 Node ---

    rviz_text = os.path.join(get_package_share_directory("Catalyst"), "urdf/Catalyst.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_text],

    )

    
# rviz2 -d ./Catalyst/urdf/Catalyst.rviz


    return LaunchDescription([
            # DeclareLaunchArgument(
            #     "use_sim_time",
            #     default_value="false",
            #     description="Use simulation (Gazebo) clock if true",
            # ),
            robot_state_publisher,
            joint_state_source,
            rviz_node,
        ])