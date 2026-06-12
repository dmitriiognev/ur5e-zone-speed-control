#!/usr/bin/env python3
"""Full-screen operator status display for the UR5e stand: zone, speed, and mode."""

import signal
import sys
from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int32

_ZONE_BACKGROUND_COLORS = ['#B71C1C', '#E65100', '#F57F17', '#558B2F', '#1B5E20']
_ZONE_LABELS = ['ZONE 0 — STOP', 'ZONE 1', 'ZONE 2', 'ZONE 3', 'ZONE 4']

ROS_SPIN_PERIOD_MS = 100


# ==================================================================================================
# View layer (PyQt5 panel; no ROS, driven by the node through setter slots)
# ==================================================================================================
class _OperatorPanel(QWidget):
    """Full-screen status panel: zone colour, speed percentage, and collaborative mode."""

    def __init__(self):
        super().__init__()
        self._zone = 0
        self._paused = False
        self._collaborative_mode = True
        self._speed_percent: Optional[int] = None

        self.setWindowTitle('UR5e Operator Panel')
        self.showFullScreen()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(24)

        self._zone_label = QLabel(alignment=Qt.AlignCenter)
        self._speed_label = QLabel(alignment=Qt.AlignCenter)
        self._mode_label = QLabel(alignment=Qt.AlignCenter)

        self._zone_label.setFont(QFont('Sans Serif', 96, QFont.Bold))
        self._speed_label.setFont(QFont('Sans Serif', 60, QFont.Bold))
        self._mode_label.setFont(QFont('Sans Serif', 40))

        layout.addStretch(1)
        layout.addWidget(self._zone_label)
        layout.addWidget(self._speed_label)
        layout.addWidget(self._mode_label)
        layout.addStretch(1)

        self._refresh()

    def set_zone(self, zone: int) -> None:
        self._zone = zone
        self._refresh()

    def set_paused(self, paused: bool) -> None:
        self._paused = paused
        self._refresh()

    def set_collaborative_mode(self, collaborative_mode: bool) -> None:
        self._collaborative_mode = collaborative_mode
        self._refresh()

    def set_speed_scaling(self, fraction: float) -> None:
        """Update the displayed speed; redraws only when the rounded percentage changes."""
        speed_percent = round(fraction * 100)
        if speed_percent != self._speed_percent:
            self._speed_percent = speed_percent
            self._refresh()

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_Escape:
            QApplication.instance().quit()

    def _refresh(self) -> None:
        if not self._collaborative_mode:
            background_color = '#1A237E'
            zone_text = 'NON-COLLABORATIVE'
            speed_text = 'AT HOME — WAITING' if self._paused else 'GOING TO HOME...'
            mode_text = 'NON-COLLABORATIVE MODE'
            mode_color = '#90CAF9'
        elif self._paused or self._zone == 0:
            background_color = _ZONE_BACKGROUND_COLORS[0]
            zone_text = 'ZONE 0 — STOP'
            speed_text = 'SPEED: 0%'
            mode_text = 'COLLABORATIVE MODE'
            mode_color = '#A5D6A7'
        else:
            index = min(self._zone, len(_ZONE_BACKGROUND_COLORS) - 1)
            background_color = _ZONE_BACKGROUND_COLORS[index]
            zone_text = _ZONE_LABELS[index] if index < len(_ZONE_LABELS) else f'ZONE {self._zone}'
            if self._speed_percent is not None:
                speed_text = f'SPEED: {self._speed_percent}%'
            else:
                speed_text = 'SPEED: —'
            mode_text = 'COLLABORATIVE MODE'
            mode_color = '#A5D6A7'

        self.setStyleSheet(f'background-color: {background_color};')
        self._zone_label.setText(zone_text)
        self._zone_label.setStyleSheet('color: white;')
        self._speed_label.setText(speed_text)
        self._speed_label.setStyleSheet('color: white;')
        self._mode_label.setText(mode_text)
        self._mode_label.setStyleSheet(f'color: {mode_color};')


# ==================================================================================================
# ROS layer (subscriptions marshalled into panel setters; ROS dependencies only here)
# ==================================================================================================
class OperatorGuiNode(Node):
    """Subscribe to zone, paused, mode, and speed topics; forward them to the attached panel."""

    def __init__(self):
        super().__init__('operator_gui_node')
        self._declare_parameters()
        self._read_parameters()
        self._panel: Optional[_OperatorPanel] = None
        self._create_interfaces()

    def _declare_parameters(self):
        """Declare every ROS parameter (defaults live in config/operator_gui.yaml)."""
        self.declare_parameter('zone_topic', '/operator/zone')
        self.declare_parameter('paused_topic', '/motion/paused')
        self.declare_parameter('collaborative_mode_topic', '/operator/collaborative_mode')
        self.declare_parameter(
            'speed_scaling_topic', '/speed_scaling_state_broadcaster/speed_scaling')
        self.declare_parameter('qos_depth', 10)

    def _read_parameters(self):
        self._zone_topic = self.get_parameter('zone_topic').value
        self._paused_topic = self.get_parameter('paused_topic').value
        self._collaborative_mode_topic = self.get_parameter('collaborative_mode_topic').value
        self._speed_scaling_topic = self.get_parameter('speed_scaling_topic').value
        self._qos_depth = self.get_parameter('qos_depth').value

    def _create_interfaces(self):
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Int32, self._zone_topic, self._zone_callback, self._qos_depth)
        self.create_subscription(Bool, self._paused_topic, self._paused_callback, latched_qos)
        self.create_subscription(
            Bool, self._collaborative_mode_topic, self._collaborative_mode_callback, latched_qos)
        self.create_subscription(
            Float64, self._speed_scaling_topic, self._speed_scaling_callback, self._qos_depth)

    def _zone_callback(self, msg: Int32) -> None:
        if self._panel is not None:
            self._panel.set_zone(msg.data)

    def _paused_callback(self, msg: Bool) -> None:
        if self._panel is not None:
            self._panel.set_paused(msg.data)

    def _collaborative_mode_callback(self, msg: Bool) -> None:
        if self._panel is not None:
            self._panel.set_collaborative_mode(msg.data)

    def _speed_scaling_callback(self, msg: Float64) -> None:
        if self._panel is not None:
            # The real UR driver reports a fraction (0..1); mock hardware reports
            # percent (100.0). Normalize so the panel always receives a fraction.
            fraction = msg.data / 100.0 if msg.data > 1.0 else msg.data
            self._panel.set_speed_scaling(fraction)

    def attach_panel(self, panel: _OperatorPanel) -> None:
        self._panel = panel


def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    node = OperatorGuiNode()

    app = QApplication(sys.argv)
    panel = _OperatorPanel()
    node.attach_panel(panel)

    # Qt owns the main loop; ROS callbacks are pumped by a timer instead of a second thread.
    def spin_ros_once():
        rclpy.spin_once(node, timeout_sec=0.0)

    ros_spin_timer = QTimer()
    ros_spin_timer.timeout.connect(spin_ros_once)
    ros_spin_timer.start(ROS_SPIN_PERIOD_MS)

    try:
        app.exec_()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
