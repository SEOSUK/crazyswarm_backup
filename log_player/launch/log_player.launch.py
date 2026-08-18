from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    log_player_share = Path(get_package_share_directory("log_player"))
    flying_pen_share = Path(get_package_share_directory("flying_pen"))
    crazyflie_share = Path(get_package_share_directory("crazyflie"))

    params_file = log_player_share / "config" / "parameters.yaml"
    rviz_config = log_player_share / "config" / "log_player_debug.rviz"
    rviz_visual_params = flying_pen_share / "config" / "rviz_visual.yaml"
    shared_su_params = crazyflie_share / "config" / "su_params.yaml"
    urdf_file = log_player_share / "models" / "cf_BLDC.urdf"

    with open(params_file, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    with open(shared_su_params, "r", encoding="utf-8") as f:
        shared_su_cfg = yaml.safe_load(f) or {}
    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description = f.read()

    csv_cfg = cfg.get("csv_playback", {}).get("ros__parameters", {})
    ui_cfg = cfg.get("log_player_control", {}).get("ros__parameters", {})
    launch_rviz = bool(csv_cfg.get("launch_rviz", False))
    su_wrench_cfg = (
        shared_su_cfg.get("robot_types", {})
        .get("cf21", {})
        .get("firmware_params", {})
        .get("su_wrench", {})
    )
    shared_ee_offset = [
        su_wrench_cfg.get("rOffX", 0.1),
        su_wrench_cfg.get("rOffY", 0.0),
        su_wrench_cfg.get("rOffZ", 0.04),
    ]

    player_params = {
        "csv_path": "",
        "start_offset_sec": float(csv_cfg.get("start_offset_sec", 0.0)),
        "playback_rate": float(csv_cfg.get("playback_rate", 1.0)),
        "sample_hz": float(csv_cfg.get("sample_hz", 50.0)),
        "publish_topic": csv_cfg.get("publish_topic", "/data_logging_msg_debug"),
        "status_topic": "/csv_player/status",
    }

    ui_params = {
        "csv_path": "",
        "playback_rate": float(csv_cfg.get("playback_rate", 1.0)),
        "status_topic": "/csv_player/status",
        "wall_x_offset": float(ui_cfg.get("wall_x_offset", 0.0)),
        "seek_step_sec": float(ui_cfg.get("seek_step_sec", 5.0)),
    }

    nodes = [
        Node(
            package="log_player",
            executable="csv_player",
            name="csv_player",
            output="screen",
            parameters=[player_params],
        ),
        Node(
            package="log_player",
            executable="log_player_control.py",
            name="log_player_control",
            output="screen",
            parameters=[ui_params],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="flying_pen",
            executable="rviz_visual",
            name="rviz_visual",
            output="screen",
            parameters=[str(rviz_visual_params), {"end_effector_offset": shared_ee_offset}],
        ),
    ]

    if launch_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
            )
        )

    return LaunchDescription(nodes)
