#!/usr/bin/env python3
from __future__ import annotations

import signal
import sys
import threading
from collections import deque
from typing import Dict, List, Tuple

import numpy as np
import pyqtgraph as pg
import rclpy
from pyqtgraph.Qt import QtCore, QtWidgets
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


FIELDS: List[str] = [
    "pose_x", "pose_y", "pose_z",
    "pose_roll", "pose_pitch", "pose_yaw",
    "f1", "f2", "f3", "f4",
    "pwm1", "pwm2", "pwm3", "pwm4",
    "bodyFx", "bodyFy", "bodyFz",
    "worldFx", "worldFy", "worldFz",
    "bodyTx", "bodyTy", "bodyTz",
    "stateVx", "stateVy", "stateVz",
    "posVx", "posVy", "posVz",
    "accWx", "accWy", "accWz",
    "zero_bias_count",
]


class DebugBuffer(Node):
    def __init__(self) -> None:
        super().__init__("data_logging_firmware_debug")
        self.declare_parameter("history_sec", 8.0)
        self.declare_parameter("update_hz", 30.0)
        self.declare_parameter("render_hz", 15.0)
        self.declare_parameter("topic", "/data_logging_msg_debug")

        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.render_hz = float(self.get_parameter("render_hz").value)
        self.topic = str(self.get_parameter("topic").value)

        self.maxlen = max(120, int(self.history_sec * self.update_hz) + 20)
        self.lock = threading.Lock()
        self.data: Dict[str, deque] = {"t": deque(maxlen=self.maxlen)}
        for key in FIELDS:
            self.data[key] = deque(maxlen=self.maxlen)
        self.latest = {key: np.nan for key in FIELDS}
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        self.create_subscription(Float64MultiArray, self.topic, self.cb_raw, 10)
        self.timer = self.create_timer(1.0 / max(self.update_hz, 1.0), self.log_snapshot)

    def cb_raw(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < len(FIELDS):
            return
        with self.lock:
            for idx, key in enumerate(FIELDS):
                self.latest[key] = float(msg.data[idx])

    def log_snapshot(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        with self.lock:
            self.data["t"].append(t)
            for key in FIELDS:
                self.data[key].append(self.latest[key])

    def get_arrays(self) -> Dict[str, np.ndarray]:
        with self.lock:
            return {k: np.array(v, dtype=float) for k, v in self.data.items()}


class PlotWindow(QtWidgets.QMainWindow):
    def __init__(self, rosbuf: DebugBuffer) -> None:
        super().__init__()
        self.rosbuf = rosbuf
        self.setWindowTitle("data_logging_firmware_debug")

        pg.setConfigOptions(antialias=True)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        self.info_label = QtWidgets.QLabel("Waiting for firmware debug data...")
        root.addWidget(self.info_label)

        grid = QtWidgets.QGridLayout()
        root.addLayout(grid)

        self.plots: Dict[str, Tuple[pg.PlotWidget, List[pg.PlotDataItem], List[str]]] = {}

        def make_plot(title: str, y_label: str, keys: List[str], colors: List[Tuple[int, int, int]]):
            widget = pg.PlotWidget()
            widget.setBackground("w")
            plot_item = widget.getPlotItem()
            plot_item.setTitle(title, color="k")
            plot_item.setLabel("bottom", "time [s]")
            plot_item.setLabel("left", y_label)
            plot_item.showGrid(x=True, y=True, alpha=0.25)
            legend = plot_item.addLegend()
            legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))
            curves = []
            for key, color in zip(keys, colors):
                curves.append(widget.plot(name=key, pen=pg.mkPen(color=color, width=2)))
            return widget, curves, keys

        self.plots["thrust"] = make_plot(
            "Uncapped Motor Thrust", "[N]",
            ["f1", "f2", "f3", "f4"],
            [(220, 50, 50), (40, 140, 70), (50, 90, 220), (160, 120, 40)],
        )
        self.plots["pwm"] = make_plot(
            "Final Actuator Ratio", "[0..65535]",
            ["pwm1", "pwm2", "pwm3", "pwm4"],
            [(220, 50, 50), (40, 140, 70), (50, 90, 220), (160, 120, 40)],
        )
        self.plots["vel_x"] = make_plot("Velocity X Compare", "[m/s]", ["stateVx", "posVx"], [(220, 50, 50), (50, 90, 220)])
        self.plots["vel_y"] = make_plot("Velocity Y Compare", "[m/s]", ["stateVy", "posVy"], [(220, 50, 50), (50, 90, 220)])
        self.plots["vel_z"] = make_plot("Velocity Z Compare", "[m/s]", ["stateVz", "posVz"], [(220, 50, 50), (50, 90, 220)])
        self.plots["acc"] = make_plot("World Acceleration", "[m/s^2]", ["accWx", "accWy", "accWz"], [(220, 50, 50), (40, 140, 70), (50, 90, 220)])
        self.plots["body_force"] = make_plot("Body Force", "[N]", ["bodyFx", "bodyFy", "bodyFz"], [(220, 50, 50), (40, 140, 70), (50, 90, 220)])
        self.plots["world_force"] = make_plot("World Force", "[N]", ["worldFx", "worldFy", "worldFz"], [(220, 50, 50), (40, 140, 70), (50, 90, 220)])
        self.plots["body_torque"] = make_plot("Body Torque", "[N*m]", ["bodyTx", "bodyTy", "bodyTz"], [(220, 50, 50), (40, 140, 70), (50, 90, 220)])

        order = ["thrust", "pwm", "vel_x", "vel_y", "vel_z", "acc", "body_force", "world_force", "body_torque"]
        for idx, name in enumerate(order):
            grid.addWidget(self.plots[name][0], idx // 3, idx % 3)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(int(1000.0 / max(self.rosbuf.render_hz, 1.0)))

    def refresh(self) -> None:
        arrays = self.rosbuf.get_arrays()
        t = arrays["t"]
        if t.size == 0:
            return
        for _name, (widget, curves, keys) in self.plots.items():
            for curve, key in zip(curves, keys):
                curve.setData(t, arrays[key])
            widget.getPlotItem().setXRange(max(0.0, t[-1] - self.rosbuf.history_sec), t[-1], padding=0.0)

        self.info_label.setText(
            f"t={t[-1]:.2f}s  thrust=({arrays['f1'][-1]:.3f}, {arrays['f2'][-1]:.3f}, {arrays['f3'][-1]:.3f}, {arrays['f4'][-1]:.3f})  "
            f"pwm=({arrays['pwm1'][-1]:.0f}, {arrays['pwm2'][-1]:.0f}, {arrays['pwm3'][-1]:.0f}, {arrays['pwm4'][-1]:.0f})"
        )


def main() -> None:
    rclpy.init()
    app = pg.mkQApp("data_logging_firmware_debug")
    rosbuf = DebugBuffer()
    window = PlotWindow(rosbuf)
    window.resize(1800, 1000)
    window.show()

    def shutdown(*_args) -> None:
        try:
            rosbuf.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        app.quit()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(rosbuf)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        exec_fn = getattr(app, "exec", None)
        if exec_fn is None:
            exec_fn = app.exec_
        exec_fn()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
