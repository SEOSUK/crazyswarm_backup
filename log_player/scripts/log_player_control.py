#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
import threading
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QStyle
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

LOG_ROOT = Path(__file__).resolve().parents[2] / "flying_pen" / "bag" / "logging"


def get_log_root() -> Path:
    return LOG_ROOT


def make_double_parameter(name: str, value: float) -> ParameterMsg:
    msg = ParameterMsg()
    msg.name = name
    msg.value.type = ParameterType.PARAMETER_DOUBLE
    msg.value.double_value = float(value)
    return msg


def make_string_parameter(name: str, value: str) -> ParameterMsg:
    msg = ParameterMsg()
    msg.name = name
    msg.value.type = ParameterType.PARAMETER_STRING
    msg.value.string_value = value
    return msg


def make_bool_parameter(name: str, value: bool) -> ParameterMsg:
    msg = ParameterMsg()
    msg.name = name
    msg.value.type = ParameterType.PARAMETER_BOOL
    msg.value.bool_value = bool(value)
    return msg


class ClickSeekSlider(QSlider):
    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            option = QStyle.sliderValueFromPosition(
                self.minimum(),
                self.maximum(),
                event.position().x() if hasattr(event, "position") else event.x(),
                max(1, self.width()),
            )
            self.setValue(int(option))
            self.sliderMoved.emit(self.value())
            self.sliderPressed.emit()
            self.sliderReleased.emit()
            event.accept()
            return
        super().mousePressEvent(event)


class LogPlayerControlNode(Node):
    def __init__(self) -> None:
        super().__init__("log_player_control")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("playback_rate", 1.0)
        self.declare_parameter("status_topic", "/csv_player/status")
        self.declare_parameter("wall_x_offset", 0.0)
        self.declare_parameter("seek_step_sec", 5.0)

        self.csv_player_client = self.create_client(SetParameters, "/csv_player/set_parameters")
        self.rviz_visual_client = self.create_client(SetParameters, "/rviz_visual/set_parameters")
        self.rviz_clear_history_client = self.create_client(Trigger, "/rviz_visual/clear_history")
        self.latest_status = {
            "loaded": False,
            "progress": 0.0,
            "current_time_sec": 0.0,
            "total_duration_sec": 0.0,
            "playback_rate": float(self.get_parameter("playback_rate").value),
            "row_count": 0.0,
            "row_index": 0.0,
            "paused": False,
        }

        status_topic = str(self.get_parameter("status_topic").value)
        self.create_subscription(Float64MultiArray, status_topic, self.on_status, 10)

    def on_status(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 7:
            return
        self.latest_status = {
            "loaded": data[0] > 0.5,
            "progress": float(data[1]),
            "current_time_sec": float(data[2]),
            "total_duration_sec": float(data[3]),
            "playback_rate": float(data[4]),
            "row_count": float(data[5]),
            "row_index": float(data[6]),
            "paused": data[7] > 0.5 if len(data) > 7 else False,
        }

    def push_initial_state(self) -> None:
        csv_path = os.path.expanduser(str(self.get_parameter("csv_path").value))
        playback_rate = float(self.get_parameter("playback_rate").value)
        wall_x_offset = float(self.get_parameter("wall_x_offset").value)
        parameters = [make_double_parameter("playback_rate", playback_rate)]
        if csv_path:
            parameters.append(make_string_parameter("csv_path", csv_path))
        self.set_csv_player_parameters(parameters)
        self.set_rviz_visual_parameters([make_double_parameter("wall_x_offset", wall_x_offset)])

    def set_csv_player_parameters(self, parameters: list[ParameterMsg]) -> None:
        if not parameters:
            return
        if not self.csv_player_client.service_is_ready():
            return
        request = SetParameters.Request()
        request.parameters = parameters
        future = self.csv_player_client.call_async(request)
        future.add_done_callback(lambda f: self._log_parameter_result("csv_player", f))

    def set_rviz_visual_parameters(self, parameters: list[ParameterMsg]) -> None:
        if not parameters:
            return
        if not self.rviz_visual_client.service_is_ready():
            return
        request = SetParameters.Request()
        request.parameters = parameters
        future = self.rviz_visual_client.call_async(request)
        future.add_done_callback(lambda f: self._log_parameter_result("rviz_visual", f))

    def seek_ratio(self, ratio: float) -> None:
        self.set_csv_player_parameters([make_double_parameter("seek_ratio", ratio)])

    def seek_time(self, time_sec: float) -> None:
        self.set_csv_player_parameters([make_double_parameter("seek_time_sec", time_sec)])

    def skip_relative(self, delta_sec: float) -> None:
        status = self.latest_status
        target_time = status["current_time_sec"] + delta_sec
        target_time = max(0.0, min(status["total_duration_sec"], target_time))
        self.seek_time(target_time)

    def play(self) -> None:
        self.set_csv_player_parameters([make_bool_parameter("paused", False)])

    def pause(self) -> None:
        self.set_csv_player_parameters([make_bool_parameter("paused", True)])

    def restart(self) -> None:
        self.set_csv_player_parameters([
            make_double_parameter("seek_ratio", 0.0),
            make_bool_parameter("paused", False),
        ])

    def clear_rviz_history(self) -> bool:
        if not self.rviz_clear_history_client.service_is_ready():
            return False
        future = self.rviz_clear_history_client.call_async(Trigger.Request())
        future.add_done_callback(self._log_clear_history_result)
        return True

    def _log_clear_history_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f"Failed to clear RViz history: {exc}")
            return
        if not response.success:
            self.get_logger().warning(f"RViz history clear failed: {response.message}")

    def _log_parameter_result(self, target_name: str, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f"Failed to update {target_name} parameters: {exc}")
            return

        for result in response.results:
            if not result.successful:
                self.get_logger().warning(f"{target_name} rejected parameter update: {result.reason}")


class LogPlayerControlWindow(QMainWindow):
    def __init__(self, ros_node: LogPlayerControlNode) -> None:
        super().__init__()
        self.ros_node = ros_node
        self.slider_is_active = False
        self.log_root = get_log_root()

        self.setWindowTitle("log_player_control")
        self.resize(820, 220)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        file_row = QHBoxLayout()
        self.file_edit = QLineEdit(os.path.expanduser(str(self.ros_node.get_parameter("csv_path").value)))
        self.file_edit.setPlaceholderText(str(self.log_root))
        browse_button = QPushButton("Browse")
        apply_button = QPushButton("Load")
        browse_button.clicked.connect(self.on_browse)
        apply_button.clicked.connect(self.on_apply_file)
        self.file_edit.returnPressed.connect(self.on_apply_file)
        file_row.addWidget(QLabel("CSV"))
        file_row.addWidget(self.file_edit, stretch=1)
        file_row.addWidget(browse_button)
        file_row.addWidget(apply_button)
        root.addLayout(file_row)

        controls = QGridLayout()
        controls.setHorizontalSpacing(12)
        controls.setVerticalSpacing(8)

        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 100.0)
        self.speed_spin.setDecimals(2)
        self.speed_spin.setSingleStep(0.1)
        self.speed_spin.setValue(float(self.ros_node.get_parameter("playback_rate").value))
        self.speed_spin.setSuffix(" x")
        self.speed_spin.valueChanged.connect(self.on_speed_changed)
        controls.addWidget(QLabel("Speed"), 0, 0)
        controls.addWidget(self.speed_spin, 0, 1)

        self.wall_x_spin = QDoubleSpinBox()
        self.wall_x_spin.setRange(-10.0, 10.0)
        self.wall_x_spin.setDecimals(3)
        self.wall_x_spin.setSingleStep(0.05)
        self.wall_x_spin.setValue(float(self.ros_node.get_parameter("wall_x_offset").value))
        self.wall_x_spin.setSuffix(" m")
        self.wall_x_spin.valueChanged.connect(self.on_wall_x_changed)
        controls.addWidget(QLabel("Wall X"), 1, 0)
        controls.addWidget(self.wall_x_spin, 1, 1)

        self.time_label = QLabel("0.00 s / 0.00 s")
        self.rows_label = QLabel("row 0 / 0")
        self.clear_history_button = QPushButton("Clear History")
        self.clear_history_button.clicked.connect(self.on_clear_history)
        controls.addWidget(QLabel("Position"), 0, 2)
        controls.addWidget(self.time_label, 0, 3)
        controls.addWidget(QLabel("Rows"), 1, 2)
        controls.addWidget(self.rows_label, 1, 3)
        controls.addWidget(self.clear_history_button, 0, 4, 2, 1)
        root.addLayout(controls)

        playback_row = QHBoxLayout()
        self.play_button = QPushButton("Play")
        self.pause_button = QPushButton("Pause")
        self.restart_button = QPushButton("Restart")
        self.back_button = QPushButton()
        self.forward_button = QPushButton()
        self.play_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.pause_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        self.restart_button.setIcon(self.style().standardIcon(QStyle.SP_MediaSkipBackward))
        self.back_button.setIcon(self.style().standardIcon(QStyle.SP_MediaSeekBackward))
        self.forward_button.setIcon(self.style().standardIcon(QStyle.SP_MediaSeekForward))
        self.play_button.clicked.connect(self.on_play)
        self.pause_button.clicked.connect(self.on_pause)
        self.restart_button.clicked.connect(self.on_restart)
        self.back_button.clicked.connect(self.on_skip_backward)
        self.forward_button.clicked.connect(self.on_skip_forward)

        self.seek_step_spin = QDoubleSpinBox()
        self.seek_step_spin.setRange(0.1, 3600.0)
        self.seek_step_spin.setDecimals(1)
        self.seek_step_spin.setSingleStep(1.0)
        self.seek_step_spin.setValue(float(self.ros_node.get_parameter("seek_step_sec").value))
        self.seek_step_spin.setSuffix(" s")
        self.seek_step_spin.valueChanged.connect(self.update_skip_button_text)
        self.update_skip_button_text(self.seek_step_spin.value())

        self.slider = ClickSeekSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.sliderPressed.connect(self.on_slider_pressed)
        self.slider.sliderReleased.connect(self.on_slider_released)
        self.slider.sliderMoved.connect(self.on_slider_moved)
        self.slider_value_label = QLabel("0.0 %")
        playback_row.addWidget(self.play_button)
        playback_row.addWidget(self.pause_button)
        playback_row.addWidget(self.restart_button)
        playback_row.addWidget(self.back_button)
        playback_row.addWidget(self.forward_button)
        playback_row.addWidget(QLabel("Step"))
        playback_row.addWidget(self.seek_step_spin)
        playback_row.addWidget(QLabel("Seek"))
        playback_row.addWidget(self.slider, stretch=1)
        playback_row.addWidget(self.slider_value_label)
        root.addLayout(playback_row)

        self.status_label = QLabel("Waiting for csv_player...")
        root.addWidget(self.status_label)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_from_status)
        self.ui_timer.start(100)

        self.player_wait_timer = QTimer(self)
        self.player_wait_timer.timeout.connect(self.try_connect_player)
        self.player_wait_timer.start(500)

    def try_connect_player(self) -> None:
        if not self.ros_node.csv_player_client.service_is_ready():
            self.ros_node.csv_player_client.wait_for_service(timeout_sec=0.0)
            return
        if not self.ros_node.rviz_visual_client.service_is_ready():
            self.ros_node.rviz_visual_client.wait_for_service(timeout_sec=0.0)
            return
        if not self.ros_node.rviz_clear_history_client.service_is_ready():
            self.ros_node.rviz_clear_history_client.wait_for_service(timeout_sec=0.0)
            return
        self.player_wait_timer.stop()
        self.ros_node.push_initial_state()
        self.status_label.setText("Connected to csv_player and rviz_visual")

    def on_browse(self) -> None:
        start_dir = str(self.log_root)
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select CSV log file",
            start_dir,
            "CSV Files (*.csv);;All Files (*)",
        )
        if not file_path:
            return
        self.file_edit.setText(file_path)
        self.on_apply_file()

    def on_apply_file(self) -> None:
        raw_value = self.file_edit.text().strip()
        csv_path = self.resolve_csv_path(raw_value)
        if not csv_path:
            self.status_label.setText(f"CSV must be inside {self.log_root}")
            return
        self.file_edit.setText(csv_path)
        self.ros_node.set_csv_player_parameters([make_string_parameter("csv_path", csv_path)])
        self.status_label.setText(f"Loading {csv_path}")

    def on_speed_changed(self, value: float) -> None:
        self.ros_node.set_csv_player_parameters([make_double_parameter("playback_rate", value)])

    def on_wall_x_changed(self, value: float) -> None:
        self.ros_node.set_rviz_visual_parameters([make_double_parameter("wall_x_offset", value)])

    def on_play(self) -> None:
        self.ros_node.play()

    def on_pause(self) -> None:
        self.ros_node.pause()

    def on_restart(self) -> None:
        self.ros_node.restart()

    def on_clear_history(self) -> None:
        if self.ros_node.clear_rviz_history():
            self.status_label.setText("Clearing RViz contact history")
        else:
            self.status_label.setText("Waiting for rviz_visual clear_history service")

    def on_skip_backward(self) -> None:
        self.ros_node.skip_relative(-self.seek_step_spin.value())

    def on_skip_forward(self) -> None:
        self.ros_node.skip_relative(self.seek_step_spin.value())

    def update_skip_button_text(self, value: float) -> None:
        step_text = f"{value:g}s"
        self.back_button.setText(f"-{step_text}")
        self.forward_button.setText(f"+{step_text}")

    def on_slider_pressed(self) -> None:
        self.slider_is_active = True

    def on_slider_moved(self, value: int) -> None:
        self.slider_value_label.setText(f"{value / 10.0:.1f} %")

    def on_slider_released(self) -> None:
        ratio = self.slider.value() / 1000.0
        self.slider_is_active = False
        self.ros_node.seek_ratio(ratio)

    def resolve_csv_path(self, raw_value: str) -> str:
        if not raw_value:
            return ""
        path = Path(os.path.expanduser(raw_value))
        if not path.is_absolute():
            path = self.log_root / path
        path = path.resolve()
        try:
            path.relative_to(self.log_root.resolve())
        except ValueError:
            return ""
        return str(path)

    def refresh_from_status(self) -> None:
        status = self.ros_node.latest_status
        current_time = status["current_time_sec"]
        total_duration = status["total_duration_sec"]
        self.time_label.setText(f"{current_time:.2f} s / {total_duration:.2f} s")
        self.rows_label.setText(f"row {int(status['row_index'])} / {int(status['row_count'])}")

        if not self.slider_is_active:
            slider_value = int(max(0.0, min(1.0, status["progress"])) * 1000.0)
            self.slider.blockSignals(True)
            self.slider.setValue(slider_value)
            self.slider.blockSignals(False)
            self.slider_value_label.setText(f"{slider_value / 10.0:.1f} %")

        if status["loaded"]:
            state_text = "Paused" if status["paused"] else "Playing"
            self.status_label.setText(
                f"{state_text} | {status['playback_rate']:.2f}x | {total_duration:.2f} s total"
            )
        else:
            self.status_label.setText("CSV not loaded")


def spin_ros(node: Node) -> None:
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def main() -> int:
    rclpy.init(args=None)
    ros_node = LogPlayerControlNode()
    spin_thread = threading.Thread(target=spin_ros, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = LogPlayerControlWindow(ros_node)
    window.show()
    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
