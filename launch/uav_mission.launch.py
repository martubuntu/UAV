from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("uav_pid_mavros")

    config_file = PathJoinSubstitution([
        pkg_share,
        "config",
        "uav_params.yaml"
    ])

    rviz_file = PathJoinSubstitution([
        pkg_share,
        "rviz",
        "uav_mission.rviz"
    ])

    mission_phase = LaunchConfiguration("mission_phase")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_offboard = LaunchConfiguration("auto_offboard")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mission_phase",
            default_value="hover_only",
            description="Mission phase: hover_only, square_only, circle_only, or full."
        ),
        DeclareLaunchArgument(
            "auto_arm",
            default_value="false",
            description="Compatibility option; this real-flight node ignores automatic arming."
        ),
        DeclareLaunchArgument(
            "auto_offboard",
            default_value="false",
            description="Compatibility option; this real-flight node ignores automatic OFFBOARD switching."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time."
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2."
        ),

        Node(
            package="uav_pid_mavros",
            executable="uav_mission_node",
            name="uav_pid_mission",
            output="screen",
            parameters=[
                config_file,
                {
                    "mission_phase": mission_phase,
                    "auto_arm": ParameterValue(auto_arm, value_type=bool),
                    "auto_offboard": ParameterValue(auto_offboard, value_type=bool),
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                }
            ]
        ),

        Node(
            condition=IfCondition(rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2_uav_mission",
            arguments=["-d", rviz_file],
            output="screen"
        )
    ])
