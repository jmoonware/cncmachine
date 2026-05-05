#!/usr/bin/env python3
"""
Reusable PyQt6 axis-control widget for a Parker 6K8 controller.

- Shared serial client is thread-safe.
- Multiple AxisWidget instances can share one Parker6KSerial object.
- Serial config: 9600 baud, 8N1, no RTS/CTS, no XON/XOFF.
- Commands and responses are logged with rotating debug logs.
"""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
import re
import sys
import threading
from dataclasses import dataclass
from typing import Optional

import serial
from PyQt6.QtCore import QObject, QRunnable, QThreadPool, QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QFont, QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


# -----------------------------
# Logging
# -----------------------------

def configure_logging(log_path: str = "parker6k_axis_widget.log") -> logging.Logger:
    logger = logging.getLogger("parker6k")
    logger.setLevel(logging.DEBUG)

    if not logger.handlers:
        handler = RotatingFileHandler(
            log_path,
            maxBytes=1_000_000,
            backupCount=10,
        )
        handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter(
            "%(asctime)s %(levelname)s [%(threadName)s] %(message)s"
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger


LOGGER = configure_logging()


# -----------------------------
# Parker serial client
# -----------------------------

class Parker6KSerial:
    """
    Thread-safe serial client for Parker 6K/6K8 controllers.

    One instance of this class should be shared by all AxisWidget instances.
    The transaction lock guarantees command/response pairs do not interleave.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 9600,
        timeout: float = 0.75,
        write_timeout: float = 0.75,
        eol: bytes = b"\r",
        read_until_prompt: bool = True,
        logger: logging.Logger = LOGGER,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.eol = eol
        self.read_until_prompt = read_until_prompt
        self.logger = logger

        self._lock = threading.RLock()
        self._ser: Optional[serial.Serial] = None

    def open(self) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                return

            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )

            # Do not assert modem-control lines on a three-wire connection.
            try:
                self._ser.setRTS(False)
                self._ser.setDTR(False)
            except Exception:
                pass

            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self.logger.info("Opened Parker serial port %s at %s baud", self.port, self.baudrate)

    def close(self) -> None:
        with self._lock:
            if self._ser:
                self._ser.close()
                self.logger.info("Closed Parker serial port %s", self.port)

    def command(self, cmd: str, *, log_poll: bool = True) -> str:
        """
        Send one command and read one complete Parker response.

        INFO-level logging suppresses TAS/TFB polling when log_poll=False,
        but DEBUG-level logging still records everything.
        """
        with self._lock:
            self.open()
            assert self._ser is not None

            clean_cmd = cmd.strip()
            is_poll = clean_cmd.endswith("TAS") or clean_cmd.endswith("TFB")

            # Always DEBUG-log all traffic.
            self.logger.debug("TX: %s", clean_cmd)

            # INFO-level skips TAS/TFB polling when requested.
            if log_poll or not is_poll:
                self.logger.info("TX: %s", clean_cmd)

            self._ser.reset_input_buffer()
            self._ser.write(clean_cmd.encode("ascii") + self.eol)
            self._ser.flush()

            response = self._read_response()

            self.logger.debug("RX: %r", response)

            if log_poll or not is_poll:
                self.logger.info("RX: %r", response)

            if response.lstrip().startswith("?"):
                self.logger.error("Parker error response to %s: %r", clean_cmd, response)

            return response

    def _read_response(self) -> str:
        """
        Parker responses commonly end with a prompt '>' plus CR/LF.
        This reads until it sees '>' or times out.
        """
        assert self._ser is not None

        chunks: list[bytes] = []

        if self.read_until_prompt:
            while True:
                b = self._ser.read(1)
                if not b:
                    break

                chunks.append(b)

                if b == b">":
                    # Consume any trailing CR/LF that arrives immediately.
                    tail = self._ser.read(2)
                    if tail:
                        chunks.append(tail)
                    break
        else:
            line = self._ser.readline()
            if line:
                chunks.append(line)

        return b"".join(chunks).decode("ascii", errors="replace").strip()


# -----------------------------
# Worker for non-blocking GUI commands
# -----------------------------

class CommandWorkerSignals(QObject):
    finished = pyqtSignal(str, str)   # command, response
    error = pyqtSignal(str, str)      # command, error_text


class CommandWorker(QRunnable):
    def __init__(self, client: Parker6KSerial, command: str, log_poll: bool = True):
        super().__init__()
        self.client = client
        self.command = command
        self.log_poll = log_poll
        self.signals = CommandWorkerSignals()

    def run(self) -> None:
        try:
            response = self.client.command(self.command, log_poll=self.log_poll)
            self.signals.finished.emit(self.command, response)
        except Exception as exc:
            LOGGER.exception("Command failed: %s", self.command)
            self.signals.error.emit(self.command, str(exc))


# -----------------------------
# GUI helpers
# -----------------------------

class StatusLight(QLabel):
    def __init__(self, label: str):
        super().__init__(label)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumWidth(120)
        self.set_on(False)

    def set_on(self, on: bool) -> None:
        color = "#18a558" if on else "#777777"
        self.setStyleSheet(
            f"""
            QLabel {{
                background-color: {color};
                color: white;
                border-radius: 10px;
                padding: 6px 10px;
                font-weight: bold;
            }}
            """
        )


class CommitSpinBox(QSpinBox):
    """
    QSpinBox already emits editingFinished on Enter and focus loss.
    This subclass disables wheel changes unless focused, which is safer
    for motion-control numeric fields.
    """

    def wheelEvent(self, event):
        if self.hasFocus():
            super().wheelEvent(event)
        else:
            event.ignore()


@dataclass
class AxisConfig:
    axis_number: int = 5
    axis_name: str = "Z"

    default_distance_steps: int = 5000
    default_velocity_rev_s: int = 1
    default_accel_rev_s2: int = 1
    default_decel_rev_s2: int = 1

    steps_per_rev: int = 8000
    thread_pitch_microns: int = 10000

    font_point_size: int = 24
    poll_interval_ms: int = 1000


# -----------------------------
# Axis widget
# -----------------------------

class AxisWidget(QWidget):
    """
    A reusable axis widget for one Parker 6K8 axis.

    Multiple instances can share the same Parker6KSerial object:

        client = Parker6KSerial("/dev/ttyUSB0")
        z = AxisWidget(client, AxisConfig(axis_number=5, axis_name="Z"))
        x = AxisWidget(client, AxisConfig(axis_number=1, axis_name="X"))
    """

    TAS_BITS = {
        1: "Moving",
        2: "NegPos",
        5: "Homed",
        12: "Stall",
        13: "Shutdown",
        14: "Fault",
        15: "PosLim",
        16: "NegLim",
        23: "PositErr",
    }

    def __init__(self, client: Parker6KSerial, config: AxisConfig | None = None):
        super().__init__()
        self.client = client
        self.config = config or AxisConfig()
        self.pool = QThreadPool.globalInstance()

        self._build_ui()
        self._connect_signals()
        self._install_shortcuts()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_status)
        self.poll_timer.start(self.config.poll_interval_ms)

        self.apply_all_settings()

    @property
    def axis(self) -> int:
        return self.config.axis_number

    @property
    def microns_per_step(self) -> float:
        return self.config.thread_pitch_microns / self.config.steps_per_rev

    def _build_ui(self) -> None:
        font = QFont()
        font.setPointSize(self.config.font_point_size)
        self.setFont(font)

        self.title = QLabel(f"Axis {self.axis}: {self.config.axis_name}")
        self.title.setStyleSheet("font-weight: bold;")

        self.distance_box = CommitSpinBox()
        self.distance_box.setRange(-999_999_999, 999_999_999)
        self.distance_box.setSingleStep(1)
        self.distance_box.setValue(self.config.default_distance_steps)

        self.physical_distance_label = QLabel()
        self._update_physical_distance_label()

        self.velocity_box = CommitSpinBox()
        self.velocity_box.setRange(0, 2_048_000)
        self.velocity_box.setValue(self.config.default_velocity_rev_s)

        self.accel_box = CommitSpinBox()
        self.accel_box.setRange(1, 39_999_998)
        self.accel_box.setValue(self.config.default_accel_rev_s2)

        self.decel_box = CommitSpinBox()
        self.decel_box.setRange(1, 39_999_998)
        self.decel_box.setValue(self.config.default_decel_rev_s2)

        self.absolute_check = QCheckBox("Absolute")
        self.home_direction_check = QCheckBox("Home Direction")
        self.drive_check = QCheckBox("Drive On")

        self.left_button = QPushButton("←")
        self.right_button = QPushButton("→")
        self.home_button = QPushButton("Home")
        self.halt_button = QPushButton("Halt")
        self.zero_button = QPushButton("Zero")

        self.position_label = QLabel("Position: --")
        self.response_label = QLabel("Response: --")
        self.response_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )

        form = QGridLayout()
        form.addWidget(QLabel("Distance, steps"), 0, 0)
        form.addWidget(self.distance_box, 0, 1)
        form.addWidget(self.physical_distance_label, 0, 2)

        form.addWidget(QLabel("Velocity, rev/s"), 1, 0)
        form.addWidget(self.velocity_box, 1, 1)

        form.addWidget(QLabel("Accel, rev/s/s"), 2, 0)
        form.addWidget(self.accel_box, 2, 1)

        form.addWidget(QLabel("Decel, rev/s/s"), 3, 0)
        form.addWidget(self.decel_box, 3, 1)

        form.addWidget(self.absolute_check, 4, 0)
        form.addWidget(self.home_direction_check, 4, 1)
        form.addWidget(self.drive_check, 4, 2)

        buttons = QHBoxLayout()
        buttons.addWidget(self.left_button)
        buttons.addWidget(self.right_button)
        buttons.addWidget(self.home_button)
        buttons.addWidget(self.halt_button)
        buttons.addWidget(self.zero_button)

        self.indicators: dict[int, StatusLight] = {}
        status_box = QGroupBox("TAS Status")
        status_grid = QGridLayout()

        for i, (bit, label) in enumerate(self.TAS_BITS.items()):
            light = StatusLight(label)
            self.indicators[bit] = light
            status_grid.addWidget(light, i // 3, i % 3)

        status_box.setLayout(status_grid)

        layout = QVBoxLayout()
        layout.addWidget(self.title)
        layout.addLayout(form)
        layout.addLayout(buttons)
        layout.addWidget(status_box)
        layout.addWidget(self.position_label)
        layout.addWidget(self.response_label)

        self.setLayout(layout)

        for button in [
            self.left_button,
            self.right_button,
            self.home_button,
            self.halt_button,
            self.zero_button,
        ]:
            button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            button.setMinimumHeight(60)

    def _connect_signals(self) -> None:
        self.distance_box.editingFinished.connect(self.on_distance_committed)
        self.velocity_box.editingFinished.connect(self.on_velocity_committed)
        self.accel_box.editingFinished.connect(self.on_accel_committed)
        self.decel_box.editingFinished.connect(self.on_decel_committed)

        self.absolute_check.toggled.connect(self.on_absolute_changed)
        self.drive_check.toggled.connect(self.on_drive_changed)

        self.left_button.clicked.connect(lambda: self.move_axis(direction=-1))
        self.right_button.clicked.connect(lambda: self.move_axis(direction=+1))
        self.home_button.clicked.connect(self.home_axis)
        self.halt_button.clicked.connect(self.halt_axis)
        self.zero_button.clicked.connect(self.zero_position)

        self.distance_box.valueChanged.connect(
            lambda _value: self._update_physical_distance_label()
        )

    def _install_shortcuts(self) -> None:
        left_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Left), self)
        left_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        left_shortcut.activated.connect(self.left_button.click)

        right_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Right), self)
        right_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        right_shortcut.activated.connect(self.right_button.click)

    def _update_physical_distance_label(self) -> None:
        microns = self.distance_box.value() * self.microns_per_step
        mm = microns / 1000.0
        self.physical_distance_label.setText(f"{microns:,.1f} µm  ({mm:,.4f} mm)")

    # -----------------------------
    # Command sending
    # -----------------------------

    def send_async(self, command: str, *, log_poll: bool = True) -> None:
        worker = CommandWorker(self.client, command, log_poll=log_poll)
        worker.signals.finished.connect(self._on_command_finished)
        worker.signals.error.connect(self._on_command_error)
        self.pool.start(worker)

    def _on_command_finished(self, command: str, response: str) -> None:
        self.response_label.setText(f"Response: {response}")

        if command.endswith("TAS"):
            self._parse_tas_response(response)
        elif command.endswith("TFB"):
            self._parse_tfb_response(response)

    def _on_command_error(self, command: str, error_text: str) -> None:
        self.response_label.setText(f"Response: ERROR during {command}: {error_text}")

    def apply_all_settings(self) -> None:
        self.on_absolute_changed(self.absolute_check.isChecked())
        self.on_distance_committed()
        self.on_velocity_committed()
        self.on_accel_committed()
        self.on_decel_committed()

    def on_distance_committed(self) -> None:
        self._update_physical_distance_label()
        self.send_async(f"{self.axis}D{self.distance_box.value()}")

    def on_velocity_committed(self) -> None:
        self.send_async(f"{self.axis}V{self.velocity_box.value()}")

    def on_accel_committed(self) -> None:
        self.send_async(f"{self.axis}A{self.accel_box.value()}")

    def on_decel_committed(self) -> None:
        self.send_async(f"{self.axis}AD{self.decel_box.value()}")

    def on_absolute_changed(self, checked: bool) -> None:
        # User requested M1 for absolute, M0 for incremental.
        self.send_async(f"{self.axis}M{1 if checked else 0}")

    def on_drive_changed(self, checked: bool) -> None:
        self.send_async(f"{self.axis}DRIVE{1 if checked else 0}")

    def move_axis(self, direction: int) -> None:
        """
        Send D with the selected sign, then GO.

        The distance box can contain a signed number, but the arrow buttons
        force the sign according to requested move direction.
        """
        magnitude = abs(self.distance_box.value())
        signed_distance = magnitude if direction > 0 else -magnitude

        # Queue as two separate serial transactions. The shared client lock keeps
        # each transaction atomic. For very strict multi-command atomicity, use
        # a compound command method; most Parker installations accept this fine.
        self.send_async(f"{self.axis}D{signed_distance}")
        self.send_async(f"{self.axis}GO")

    def home_axis(self) -> None:
        direction = 1 if self.home_direction_check.isChecked() else 0
        self.send_async(f"{self.axis}HOM{direction}")

    def halt_axis(self) -> None:
        # User requested: axis 5 => !K00001000.
        self.send_async(f"!K{self._axis_bitmask()}")

    def zero_position(self) -> None:
        self.send_async(f"{self.axis}PSET0")

    def poll_status(self) -> None:
        self.send_async(f"{self.axis}TAS", log_poll=False)
        self.send_async(f"{self.axis}TFB", log_poll=False)

    # -----------------------------
    # Parsing
    # -----------------------------

    def _axis_bitmask(self) -> str:
        """
        Parker 8-axis bit field, left-to-right axes 1..8.
        Axis 5 -> 00001000.
        """
        bits = ["0"] * 8
        if not 1 <= self.axis <= 8:
            raise ValueError("Axis number must be in range 1..8")
        bits[self.axis - 1] = "1"
        return "".join(bits)

    def _parse_tas_response(self, response: str) -> None:
        """
        Expected examples:
            *1TAS0000_0000_0000_0000_0000_0000_0000_0000
            *TAS 0000_... repeated for active axes

        For single-axis polling, use the first 32-bit binary field found.
        """
        fields = re.findall(r"[01_]{32,}", response)
        if not fields:
            return

        bit_string = fields[0].replace("_", "")

        if len(bit_string) < 32:
            return

        for bit, light in self.indicators.items():
            # Parker bit numbering is left-to-right, 1-based.
            value = bit_string[bit - 1] == "1"
            light.set_on(value)

    def _parse_tfb_response(self, response: str) -> None:
        """
        Expected examples vary by firmware/configuration, but usually contain
        one numeric position after the TFB command name.
        """
        cleaned = response.replace("\r", " ").replace("\n", " ")

        # Prefer the number after TFB, if present.
        match = re.search(r"TFB\s*=?\s*(-?\d+(?:\.\d+)?)", cleaned)
        if not match:
            nums = re.findall(r"-?\d+(?:\.\d+)?", cleaned)
            if nums:
                # Avoid picking the axis number from '*5TFB...' when possible.
                value = nums[-1]
            else:
                return
        else:
            value = match.group(1)

        self.position_label.setText(f"Position: {value}")


# -----------------------------
# Demo application
# -----------------------------

def main() -> int:
    app = QApplication(sys.argv)

    client = Parker6KSerial(
        port="/dev/ttyUSB0",
        baudrate=9600,
        timeout=0.75,
        write_timeout=0.75,
    )

    widget = AxisWidget(
        client,
        AxisConfig(
            axis_number=5,
            axis_name="Z",
            default_distance_steps=5000,
            steps_per_rev=8000,
            thread_pitch_microns=10000,
            font_point_size=24,
        ),
    )

    widget.setWindowTitle("Parker 6K8 Axis Control")
    widget.resize(1000, 700)
    widget.show()

    exit_code = app.exec()
    client.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
