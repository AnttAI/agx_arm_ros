from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def _declare_arm_arguments(side: str):
    return [
        DeclareLaunchArgument(
            f"{side}_can_port",
            default_value="can0" if side == "left" else "can1",
            description=f"CAN port for the {side} arm.",
        ),
        DeclareLaunchArgument(
            f"{side}_arm_type",
            default_value="piper",
            description=f"Robot type for the {side} arm.",
            choices=["piper", "nero", "piper_x", "piper_h", "piper_l"],
        ),
        DeclareLaunchArgument(
            f"{side}_effector_type",
            default_value="none",
            description=f"End effector type for the {side} arm.",
            choices=["none", "agx_gripper", "revo2"],
        ),
        DeclareLaunchArgument(
            f"{side}_auto_enable",
            default_value="True",
            description=f"Automatically enable the {side} arm.",
        ),
        DeclareLaunchArgument(
            f"{side}_installation_pos",
            default_value="horizontal",
            description=f"Installation position for the {side} arm.",
            choices=["horizontal", "left", "right"],
        ),
        DeclareLaunchArgument(
            f"{side}_speed_percent",
            default_value="100",
            description=f"Motion speed percentage for the {side} arm.",
        ),
        DeclareLaunchArgument(
            f"{side}_pub_rate",
            default_value="200",
            description=f"Feedback publish rate for the {side} arm.",
        ),
        DeclareLaunchArgument(
            f"{side}_enable_timeout",
            default_value="5.0",
            description=f"Enable timeout in seconds for the {side} arm.",
        ),
        DeclareLaunchArgument(
            f"{side}_payload",
            default_value="empty",
            description=f"Payload profile for the {side} arm.",
            choices=["empty", "half", "full"],
        ),
        DeclareLaunchArgument(
            f"{side}_tcp_offset",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            description=f"TCP offset for the {side} arm.",
        ),
    ]


def _make_arm_node(side: str):
    side_ns = f"/{side}_arm"

    return Node(
        package="agx_arm_ctrl",
        executable="agx_arm_ctrl_single",
        name=f"{side}_agx_arm_ctrl_single_node",
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "can_port": LaunchConfiguration(f"{side}_can_port"),
            "pub_rate": LaunchConfiguration(f"{side}_pub_rate"),
            "auto_enable": LaunchConfiguration(f"{side}_auto_enable"),
            "arm_type": LaunchConfiguration(f"{side}_arm_type"),
            "speed_percent": LaunchConfiguration(f"{side}_speed_percent"),
            "enable_timeout": LaunchConfiguration(f"{side}_enable_timeout"),
            "installation_pos": LaunchConfiguration(f"{side}_installation_pos"),
            "effector_type": LaunchConfiguration(f"{side}_effector_type"),
            "payload": LaunchConfiguration(f"{side}_payload"),
            "tcp_offset": LaunchConfiguration(f"{side}_tcp_offset"),
        }],
        remappings=[
            ("/feedback/joint_states", f"{side_ns}/feedback/joint_states"),
            ("/feedback/tcp_pose", f"{side_ns}/feedback/tcp_pose"),
            ("/feedback/arm_status", f"{side_ns}/feedback/arm_status"),
            ("/feedback/arm_ctrl_states", f"{side_ns}/feedback/arm_ctrl_states"),
            ("/feedback/gripper_status", f"{side_ns}/feedback/gripper_status"),
            ("/feedback/hand_status", f"{side_ns}/feedback/hand_status"),
            ("/feedback/leader_joint_angles", f"{side_ns}/feedback/leader_joint_angles"),
            ("/control/joint_states", f"{side_ns}/control/joint_states"),
            ("/control/move_j", f"{side_ns}/control/move_j"),
            ("/control/move_p", f"{side_ns}/control/move_p"),
            ("/control/move_l", f"{side_ns}/control/move_l"),
            ("/control/move_c", f"{side_ns}/control/move_c"),
            ("/control/move_mit", f"{side_ns}/control/move_mit"),
            ("/control/move_js", f"{side_ns}/control/move_js"),
            ("/control/gripper", f"{side_ns}/control/gripper"),
            ("/control/hand", f"{side_ns}/control/hand"),
            ("/control/hand_position_time", f"{side_ns}/control/hand_position_time"),
            ("/enable_agx_arm", f"{side_ns}/enable_agx_arm"),
            ("/move_home", f"{side_ns}/move_home"),
            ("/exit_teach_mode", f"{side_ns}/exit_teach_mode"),
        ],
    )


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal).",
    )

    base_arguments = [
        DeclareLaunchArgument(
            "start_tara_base",
            default_value="True",
            description="Start the Tara wheel-base ROS node.",
        ),
        DeclareLaunchArgument("base_serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("base_slave_id", default_value="1"),
        DeclareLaunchArgument("base_baudrate", default_value="115200"),
        DeclareLaunchArgument("base_wheel_radius_m", default_value="0.10"),
        DeclareLaunchArgument("base_wheel_separation_m", default_value="0.157"),
        DeclareLaunchArgument("base_max_abs_rpm", default_value="30.0"),
        DeclareLaunchArgument("base_command_timeout_s", default_value="0.5"),
    ]

    base_node = Node(
        package="tara_base_ctrl",
        executable="tara_base_node",
        name="tara_base",
        namespace="base",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_tara_base")),
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "serial_port": LaunchConfiguration("base_serial_port"),
            "slave_id": LaunchConfiguration("base_slave_id"),
            "baudrate": LaunchConfiguration("base_baudrate"),
            "wheel_radius_m": LaunchConfiguration("base_wheel_radius_m"),
            "wheel_separation_m": LaunchConfiguration("base_wheel_separation_m"),
            "max_abs_rpm": LaunchConfiguration("base_max_abs_rpm"),
            "command_timeout_s": LaunchConfiguration("base_command_timeout_s"),
        }],
    )

    return LaunchDescription(
        [log_level_arg]
        + _declare_arm_arguments("left")
        + _declare_arm_arguments("right")
        + base_arguments
        + [
            _make_arm_node("left"),
            _make_arm_node("right"),
            base_node,
        ]
    )
