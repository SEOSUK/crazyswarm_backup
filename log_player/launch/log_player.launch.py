from pathlib import Path
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def _expand(path_str: str) -> str:
    return os.path.expanduser(path_str)


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("log_player"))
    params_file = pkg_share / "config" / "parameters.yaml"
    wrench_params = pkg_share / "config" / "wrench_observer.yaml"
    normal_params = pkg_share / "config" / "normal_vector_estimation.yaml"
    normal_force_panel_params = pkg_share / "config" / "normal_vector_force_panel.yaml"
    mob_consistency_panel_params = pkg_share / "config" / "mob_consistency_panel.yaml"
    rviz_config = pkg_share / "config" / "log_player.rviz"
    urdf_file = pkg_share / "models" / "cf_BLDC.urdf"

    with open(params_file, "r", encoding="utf-8") as f:
      cfg = yaml.safe_load(f)

    csv_cfg = cfg.get("csv_playback", {}).get("ros__parameters", {})
    panel_cfg = cfg.get("panel", {}).get("ros__parameters", {})
    launch_rviz = bool(csv_cfg.get("launch_rviz", True))

    with open(urdf_file, "r", encoding="utf-8") as f:
      robot_description = f.read()

    shared_params = [{"use_sim_time": False}]

    player_params = {
        "csv_path": "",
        "start_offset_sec": float(csv_cfg.get("start_offset_sec", 0.0)),
        "playback_rate": float(csv_cfg.get("playback_rate", 1.0)),
        "publish_topic": csv_cfg.get("publish_topic", "/data_logging_msg"),
        "status_topic": "/csv_player/status",
    }

    ui_params = {
        "csv_path": "",
        "playback_rate": float(csv_cfg.get("playback_rate", 1.0)),
        "status_topic": "/csv_player/status",
        "wall_x_offset": 0.0,
    }

    panel_runtime_params = {
        key: panel_cfg[key]
        for key in ("history_sec", "update_hz", "render_hz")
        if key in panel_cfg
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
            parameters=shared_params + [{"robot_description": robot_description}],
        ),
        Node(
            package="log_player",
            executable="log_decoder",
            name="log_decoder",
            output="screen",
            parameters=[str(params_file)],
        ),
        Node(
            package="log_player",
            executable="rviz_visual",
            name="rviz_visual",
            output="screen",
            parameters=[str(params_file)],
        ),
        Node(
            package="log_player",
            executable="wrench_observer",
            name="wrench_observer",
            output="screen",
            parameters=[str(wrench_params)],
        ),
        Node(
            package="log_player",
            executable="normal_vector_estimation",
            name="normal_vector_estimation_pure",
            output="screen",
            parameters=[str(normal_params), {
                "force_observation_source": "mob_2nd",
                "contact_frame_quat_topic": "/log_player/offline/contact_frame_quat_pure",
                "contact_force_x_topic": "/log_player/offline/contact_force_x_pure",
                "normal_debug_metrics_topic": "/log_player/offline/normal_debug_metrics_pure",
            }],
        ),
        Node(
            package="log_player",
            executable="normal_vector_estimation",
            name="normal_vector_estimation_k1",
            output="screen",
            parameters=[str(normal_params), {
                "force_observation_source": "mob_2nd_tau",
                "contact_frame_quat_topic": "/log_player/offline/contact_frame_quat",
                "contact_force_x_topic": "/log_player/offline/contact_force_x",
                "normal_debug_metrics_topic": "/log_player/offline/normal_debug_metrics",
            }],
        ),
        Node(
            package="log_player",
            executable="normal_vector_estimation",
            name="normal_vector_estimation_k1_novcorr",
            output="screen",
            parameters=[str(normal_params), {
                "force_observation_source": "mob_2nd_tau",
                "contact_frame_quat_topic": "/log_player/offline/contact_frame_quat_k1_novcorr",
                "contact_force_x_topic": "/log_player/offline/contact_force_x_k1_novcorr",
                "normal_debug_metrics_topic": "/log_player/offline/normal_debug_metrics_k1_novcorr",
                "normal_force_based.epsilon_g": 1.0e9,
            }],
        ),
        Node(
            package="log_player",
            executable="normal_vector_estimation",
            name="normal_vector_estimation_k2",
            output="screen",
            parameters=[str(normal_params), {
                "force_observation_source": "mob_2nd_tau_ke2",
                "contact_frame_quat_topic": "/log_player/offline/contact_frame_quat_k2",
                "contact_force_x_topic": "/log_player/offline/contact_force_x_k2",
                "normal_debug_metrics_topic": "/log_player/offline/normal_debug_metrics_k2",
            }],
        ),
        Node(
            package="log_player",
            executable="normal_vector_estimation",
            name="normal_vector_estimation_k2_nolpf",
            output="screen",
            parameters=[str(normal_params), {
                "force_observation_source": "mob_2nd_tau_ke2",
                "contact_frame_quat_topic": "/log_player/offline/contact_frame_quat_k2_nolpf",
                "contact_force_x_topic": "/log_player/offline/contact_force_x_k2_nolpf",
                "normal_debug_metrics_topic": "/log_player/offline/normal_debug_metrics_k2_nolpf",
                "normal_force_based.candidate_lpf_alpha": 0.0,
            }],
        ),
        Node(
            package="log_player",
            executable="normal_vector_estimation",
            name="normal_vector_estimation_k2_novcorr",
            output="screen",
            parameters=[str(normal_params), {
                "force_observation_source": "mob_2nd_tau_ke2",
                "contact_frame_quat_topic": "/log_player/offline/contact_frame_quat_k2_novcorr",
                "contact_force_x_topic": "/log_player/offline/contact_force_x_k2_novcorr",
                "normal_debug_metrics_topic": "/log_player/offline/normal_debug_metrics_k2_novcorr",
                "normal_force_based.epsilon_g": 1.0e9,
            }],
        ),
        Node(
            package="log_player",
            executable="offline_result_logger",
            name="offline_result_logger",
            output="screen",
        ),
    ]

    if launch_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                parameters=shared_params,
                arguments=["-d", str(rviz_config)],
            )
        )

    enabled_panels = panel_cfg.get("enabled_panels", [])
    if not enabled_panels:
        selected = str(panel_cfg.get("selected", "")).strip()
        if selected:
            enabled_panels = [selected]

    panel_param_files = {
        "normal_vector_force": normal_force_panel_params,
        "mob_consistency": mob_consistency_panel_params,
    }

    for panel_name in enabled_panels:
        panel_params = panel_param_files.get(panel_name)
        if panel_params is None:
            continue
        nodes.append(
            TimerAction(
                period=float(panel_cfg.get("delay_sec", 1.5)),
                actions=[
                    Node(
                        package="flyingpen_plotter",
                        executable=panel_name,
                        name=panel_name,
                        output="screen",
                        parameters=[
                            str(params_file),
                            str(wrench_params),
                            str(normal_params),
                            str(panel_params),
                            panel_runtime_params,
                        ],
                    )
                ],
            )
        )

    return LaunchDescription(nodes)
