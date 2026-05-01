#!/usr/bin/env python3
"""
PyQt6 IDS peak USB monochrome camera control widget.

Target: Raspberry Pi 4 + IDS 3680XLE-M monochrome USB camera.

This version does NOT free-run the camera in a worker thread.  It configures
software triggering and uses a Qt timer to request one frame at a time:

    init_software_trigger()
    start_acquisition()
    make_([1stvision.com](https://www.1stvision.com/cameras/IDS/IDS-manuals/en/program-start-acquisition.html?utm_source=chatgpt.com))slower, but much easier to debug on a Raspberry Pi 4 than
continuous USB streaming.
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Any, cast

import numpy as np

from PyQt6.QtCore import Qt, pyqtSignal, QPointF, QTimer
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from ids_peak import ids_peak

try:
    from ids_peak import ids_peak_ipl_extension
    from ids_peak_ipl import ids_peak_ipl
except Exception:
    ids_peak_ipl_extension = None
    ids_peak_ipl = None


# ------------------------------- Utilities ---------------------------------


def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def clamp_float(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def normalize_to_u8(image: np.ndarray) -> np.ndarray:
    """Return an 8-bit view suitable for display."""
    if image.ndim != 2:
        raise ValueError("Expected a 2-D monochrome image array")

    if image.dtype == np.uint8:
        return np.ascontiguousarray(image)

    arr = image.astype(np.float32, copy=False)
    min_v = float(np.nanmin(arr))
    max_v = float(np.nanmax(arr))
    if max_v <= min_v:
        return np.zeros(image.shape, dtype=np.uint8)

    scaled = (arr - min_v) * (255.0 / (max_v - min_v))
    return np.ascontiguousarray(np.clip(scaled, 0, 255).astype(np.uint8))


def numpy_gray_to_qimage(image: np.ndarray) -> QImage:
    img8 = normalize_to_u8(image)
    h, w = img8.shape
    qimg = QImage(img8.data, w, h, img8.strides[0], QImage.Format.Format_Grayscale8)
    return qimg.copy()


def save_numpy_image(path: str, image: np.ndarray) -> None:
    try:
        from PIL import Image

        arr = np.ascontiguousarray(image)
        if arr.dtype == np.uint8:
            im = Image.fromarray(arr, mode="L")
        elif arr.dtype == np.uint16:
            im = Image.fromarray(arr, mode="I;16")
        else:
            im = Image.fromarray(arr.astype(np.uint16), mode="I;16")
        im.save(path)
    except ImportError:
        numpy_gray_to_qimage(image).save(path)


def set_enum_if_available(nodemap: Any, node_name: str, entry_name: str) -> bool:
    try:
        node = cast(ids_peak.EnumerationNode, nodemap.FindNode(node_name))
        node.SetCurrentEntry(entry_name)
        return True
    except Exception:
        return False


def set_boolean_if_available(nodemap: Any, node_name: str, value: bool) -> bool:
    try:
        node = cast(ids_peak.BooleanNode, nodemap.FindNode(node_name))
        node.SetValue(value)
        return True
    except Exception:
        return False


def execute_command(nodemap: Any, node_name: str, wait: bool = True) -> None:
    node = cast(ids_peak.CommandNode, nodemap.FindNode(node_name))
    node.Execute()
    if wait:
        node.WaitUntilDone()


def set_float_node(nodemap: Any, node_name: str, value: float) -> None:
    node = cast(ids_peak.FloatNode, nodemap.FindNode(node_name))
    lo = node.Minimum()
    hi = node.Maximum()
    node.SetValue(float(max(lo, min(hi, value))))


def get_float_node_limits(nodemap: Any, node_name: str, default: Tuple[float, float]) -> Tuple[float, float]:
    try:
        node = cast(ids_peak.FloatNode, nodemap.FindNode(node_name))
        return float(node.Minimum()), float(node.Maximum())
    except Exception:
        return default


# -------------------------- IDS peak camera wrapper --------------------------


@dataclass
class CameraSettings:
    exposure_us: float = 5000.0
    gain: float = 1.0


class IDSPeakMonoCamera:
    """IDS peak wrapper using software-triggered single-frame acquisition."""

    def __init__(self) -> None:
        self.device = None
        self.nodemap = None
        self.datastream = None
        self._opened = False
        self._acquiring = False

    def open(self) -> None:
        if self._opened:
            return

        ids_peak.Library.Initialize()

        device_manager = ids_peak.DeviceManager.Instance()
        device_manager.Update()

        devices = device_manager.Devices()
        if len(devices) == 0:
            ids_peak.Library.Close()
            raise RuntimeError("No IDS peak device found")

        selected = None
        for dev in devices:
            if dev.IsOpenable():
                selected = dev
                break
        if selected is None:
            ids_peak.Library.Close()
            raise RuntimeError("No openable IDS peak device found")

        self.device = selected.OpenDevice(ids_peak.DeviceAccessType_Control)
        datastreams = self.device.DataStreams()
        if len(datastreams) == 0:
            self.close()
            raise RuntimeError("Camera has no DataStream")

        self.datastream = datastreams[0].OpenDataStream()
        self.nodemap = self.device.RemoteDevice().NodeMaps()[0]

        try:
            cast(ids_peak.EnumerationNode, self.nodemap.FindNode("UserSetSelector")).SetCurrentEntry("Default")
            execute_command(self.nodemap, "UserSetLoad")
        except Exception:
            pass

        # Make the payload as simple as possible.
        set_enum_if_available(self.nodemap, "PixelFormat", "Mono8")
        set_enum_if_available(self.nodemap, "AcquisitionMode", "Continuous")

        self._allocate_buffers(buffer_count=8)
        self.init_software_trigger()
        self._opened = True

    def _allocate_buffers(self, buffer_count: int = 8) -> None:
        if self.nodemap is None or self.datastream is None:
            raise RuntimeError("Camera not open")

        payload_size = cast(ids_peak.IntegerNode, self.nodemap.FindNode("PayloadSize")).Value()
        n_buffers = max(buffer_count, self.datastream.NumBuffersAnnouncedMinRequired())

        for _ in range(n_buffers):
            buffer = self.datastream.AllocAndAnnounceBuffer(payload_size)
            self.datastream.QueueBuffer(buffer)

    def close(self) -> None:
        try:
            self.stop_acquisition()
        except Exception:
            pass

        if self.datastream is not None:
            try:
                for buffer in self.datastream.AnnouncedBuffers():
                    self.datastream.RevokeBuffer(buffer)
            except Exception:
                pass

        self.datastream = None
        self.nodemap = None
        self.device = None

        if self._opened:
            try:
                ids_peak.Library.Close()
            except Exception:
                pass
        self._opened = False

    def init_software_trigger(self) -> None:
        """Configure FrameStart to wait for TriggerSoftware."""
        if self.nodemap is None:
            raise RuntimeError("Camera not open")

        # Disable trigger first so selector/source can be changed safely.
        set_enum_if_available(self.nodemap, "TriggerSelector", "FrameStart")
        set_enum_if_available(self.nodemap, "TriggerMode", "Off")
        set_enum_if_available(self.nodemap, "TriggerSource", "Software")
        set_enum_if_available(self.nodemap, "TriggerActivation", "RisingEdge")
        set_enum_if_available(self.nodemap, "TriggerMode", "On")

    def start_acquisition(self) -> None:
        """Start DataStream and camera acquisition once; frames still require triggers."""
        if self.nodemap is None or self.datastream is None:
            raise RuntimeError("Camera not open")
        if self._acquiring:
            return

        try:
            self.datastream.Flush(ids_peak.DataStreamFlushMode_DiscardAll)
            for buffer in self.datastream.AnnouncedBuffers():
                self.datastream.QueueBuffer(buffer)
        except Exception:
            pass

        try:
            cast(ids_peak.IntegerNode, self.nodemap.FindNode("TLParamsLocked")).SetValue(1)
        except Exception:
            pass

        self.datastream.StartAcquisition()
        execute_command(self.nodemap, "AcquisitionStart")
        self._acquiring = True

    def stop_acquisition(self) -> None:
        if not self._acquiring:
            return
        if self.nodemap is None or self.datastream is None:
            return

        try:
            execute_command(self.nodemap, "AcquisitionStop")
        except Exception:
            pass
        try:
            self.datastream.KillWait()
        except Exception:
            pass
        try:
            self.datastream.StopAcquisition(ids_peak.AcquisitionStopMode_Default)
            self.datastream.Flush(ids_peak.DataStreamFlushMode_DiscardAll)
        except Exception:
            pass
        try:
            cast(ids_peak.IntegerNode, self.nodemap.FindNode("TLParamsLocked")).SetValue(0)
        except Exception:
            pass
        self._acquiring = False

    # Backwards-compatible aliases for older GUI names.
    start_streaming = start_acquisition
    stop_streaming = stop_acquisition

    def set_exposure_us(self, exposure_us: float) -> None:
        if self.nodemap is not None:
            set_float_node(self.nodemap, "ExposureTime", exposure_us)

    def set_gain(self, gain: float) -> None:
        if self.nodemap is not None:
            set_float_node(self.nodemap, "Gain", gain)

    def gain_limits(self) -> Tuple[float, float]:
        if self.nodemap is None:
            return 0.0, 24.0
        return get_float_node_limits(self.nodemap, "Gain", (0.0, 24.0))

    def exposure_limits(self) -> Tuple[float, float]:
        if self.nodemap is None:
            return 1.0, 10_000_000.0
        return get_float_node_limits(self.nodemap, "ExposureTime", (1.0, 10_000_000.0))

    def _buffer_to_numpy(self, buffer: Any) -> np.ndarray:
        if ids_peak_ipl_extension is not None and ids_peak_ipl is not None:
            image = ids_peak_ipl_extension.BufferToImage(buffer)
            try:
                image = image.ConvertTo(
                    ids_peak_ipl.PixelFormatName_Mono8,
                    ids_peak_ipl.ConversionMode_Fast,
                )
            except Exception:
                image = image.ConvertTo(
                    ids_peak_ipl.PixelFormatName.Mono8,
                    ids_peak_ipl.ConversionMode.Fast,
                )
            arr = image.get_numpy_2D()
        else:
            image_view = buffer.ToImageView()
            arr = image_view.to_numpy_array()

        arr = np.asarray(arr)
        if arr.ndim == 3:
            arr = arr[:, :, 0]
        if arr.ndim != 2:
            raise RuntimeError(f"Expected 2-D monochrome image, got shape {arr.shape}")
        return np.ascontiguousarray(arr.copy())

    def make_image(self, timeout_ms: int = 5000) -> np.ndarray:
        """Issue one software trigger and return one 2-D NumPy frame."""
        if self.nodemap is None or self.datastream is None:
            raise RuntimeError("Camera not open")

        self.start_acquisition()

        execute_command(self.nodemap, "TriggerSoftware", wait=False)
        buffer = self.datastream.WaitForFinishedBuffer(ids_peak.Timeout(timeout_ms))
        try:
            # Do not use IsIncomplete here.  On the problem Raspberry Pi path it
            # throws/flags even when the apparent NumPy shape and byte count are
            # correct.  Debug the image payload itself first.
            return self._buffer_to_numpy(buffer)
        finally:
            self.datastream.QueueBuffer(buffer)

    def acquire_frame(self, timeout_ms: int = 5000) -> np.ndarray:
        return self.make_image(timeout_ms=timeout_ms)

    def acquire_single(self, exposure_us: float = 5000.0, gain: float = 1.0, timeout_ms: int = 5000) -> np.ndarray:
        self.set_exposure_us(exposure_us)
        self.set_gain(gain)
        time.sleep(max(0.002, exposure_us / 1_000_000.0))
        return self.make_image(timeout_ms=timeout_ms)


# ------------------------------ GUI widgets ---------------------------------


class CameraImageWidget(QLabel):
    cursor_changed = pyqtSignal(int, int)
    origin_changed = pyqtSignal(int, int)

    def __init__(self) -> None:
        super().__init__()
        self.setMouseTracking(True)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(640, 480)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background: #111; color: #ccc;")
        self.setText("No image")
        self.image: Optional[np.ndarray] = None
        self.qimage: Optional[QImage] = None
        self.cursor_x = 0
        self.cursor_y = 0
        self.origin_x = 0
        self.origin_y = 0

    def set_image(self, image: np.ndarray) -> None:
        self.image = image
        self.qimage = numpy_gray_to_qimage(image)
        h, w = image.shape
        self.cursor_x = clamp(self.cursor_x, 0, w - 1)
        self.cursor_y = clamp(self.cursor_y, 0, h - 1)
        self.update()

    def image_to_widget_rect(self) -> Tuple[int, int, int, int]:
        if self.qimage is None:
            return 0, 0, self.width(), self.height()
        pix = QPixmap.fromImage(self.qimage)
        scaled = pix.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        x0 = (self.width() - scaled.width()) // 2
        y0 = (self.height() - scaled.height()) // 2
        return x0, y0, scaled.width(), scaled.height()

    def widget_to_image_xy(self, pos: QPointF) -> Optional[Tuple[int, int]]:
        if self.image is None:
            return None
        h, w = self.image.shape
        x0, y0, rw, rh = self.image_to_widget_rect()
        if rw <= 0 or rh <= 0:
            return None
        x = int(round((pos.x() - x0) * (w - 1) / max(1, rw - 1)))
        y = int(round((pos.y() - y0) * (h - 1) / max(1, rh - 1)))
        if x < 0 or x >= w or y < 0 or y >= h:
            return None
        return x, y

    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
        xy = self.widget_to_image_xy(event.position())
        if xy is None:
            return
        self.cursor_x, self.cursor_y = xy
        self.cursor_changed.emit(self.cursor_x, self.cursor_y)
        self.update()

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        xy = self.widget_to_image_xy(event.position())
        if xy is None:
            return
        self.cursor_x, self.cursor_y = xy
        if event.button() == Qt.MouseButton.RightButton:
            self.origin_x = self.cursor_x
            self.origin_y = self.cursor_y
            self.origin_changed.emit(self.origin_x, self.origin_y)
        self.cursor_changed.emit(self.cursor_x, self.cursor_y)
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        super().paintEvent(event)
        if self.qimage is None:
            return
        painter = QPainter(self)
        pix = QPixmap.fromImage(self.qimage)
        scaled = pix.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        x0 = (self.width() - scaled.width()) // 2
        y0 = (self.height() - scaled.height()) // 2
        painter.drawPixmap(x0, y0, scaled)

        if self.image is not None:
            h, w = self.image.shape
            sx = scaled.width() / max(1, w)
            sy = scaled.height() / max(1, h)
            cx = int(x0 + (self.cursor_x + 0.5) * sx)
            cy = int(y0 + (self.cursor_y + 0.5) * sy)
            painter.setPen(QPen(QColor(255, 60, 60), 1))
            painter.drawLine(cx, y0, cx, y0 + scaled.height())
            painter.drawLine(x0, cy, x0 + scaled.width(), cy)
        painter.end()


class CaptureWindow(QMainWindow):
    def __init__(self, image: np.ndarray, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.image = np.array(image, copy=True)
        self.setWindowTitle("Captured Image")
        central = QWidget()
        layout = QVBoxLayout(central)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setPixmap(QPixmap.fromImage(numpy_gray_to_qimage(self.image)))
        self.image_label.setMinimumSize(640, 480)
        layout.addWidget(self.image_label)
        save_btn = QPushButton("Save Image...")
        save_btn.clicked.connect(self.save_image)
        layout.addWidget(save_btn)
        self.setCentralWidget(central)

    def save_image(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Image",
            os.path.join(os.getcwd(), "image.png"),
            "Images (*.png *.tif *.tiff);;All files (*)",
        )
        if path:
            save_numpy_image(path, self.image)


class USBCameraControlWidget(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IDS 3680XLE-M USB Camera Control")
        self.camera = IDSPeakMonoCamera()
        self.current_image: Optional[np.ndarray] = None
        self.capture_windows: list[CaptureWindow] = []
        self.scale_enabled = False
        self.microns_per_pixel = 5.0
        self.acquiring_image = False

        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.on_timer)

        self._build_ui()
        self._open_camera()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)

        top = QHBoxLayout()
        self.image_widget = CameraImageWidget()
        self.image_widget.cursor_changed.connect(self.on_cursor_changed)
        self.image_widget.origin_changed.connect(self.on_origin_changed)
        top.addWidget(self.image_widget, stretch=3)

        plot_box = QVBoxLayout()
        self.h_plot = pg.PlotWidget(title="Horizontal line slice")
        self.v_plot = pg.PlotWidget(title="Vertical line slice")
        self.h_curve = self.h_plot.plot([], [])
        self.v_curve = self.v_plot.plot([], [])
        self.h_plot.setLabel("left", "Intensity", units="DN")
        self.v_plot.setLabel("left", "Intensity", units="DN")
        self.h_plot.setLabel("bottom", "X", units="pixels")
        self.v_plot.setLabel("bottom", "Y", units="pixels")
        plot_box.addWidget(self.h_plot)
        plot_box.addWidget(self.v_plot)
        top.addLayout(plot_box, stretch=2)
        root.addLayout(top, stretch=1)

        self.position_label = QLabel("Cursor: X=0 px, Y=0 px   Origin: X0=0, Y0=0")
        root.addWidget(self.position_label)

        controls = QGroupBox("Camera Controls")
        grid = QGridLayout(controls)

        self.start_stop = QCheckBox("Start timer acquisition")
        self.start_stop.toggled.connect(self.on_start_stop)
        grid.addWidget(self.start_stop, 0, 0, 1, 2)

        capture_btn = QPushButton("Capture")
        capture_btn.clicked.connect(self.capture_single_window)
        grid.addWidget(capture_btn, 0, 2)

        form = QFormLayout()
        self.exposure_spin = QDoubleSpinBox()
        self.exposure_spin.setDecimals(0)
        self.exposure_spin.setSingleStep(1.0)
        self.exposure_spin.setRange(1, 10_000_000)
        self.exposure_spin.setValue(5000)
        self.exposure_spin.setSuffix(" µs")
        self.exposure_spin.valueChanged.connect(self.on_exposure_changed)
        form.addRow("Exposure:", self.exposure_spin)

        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setDecimals(1)
        self.gain_spin.setSingleStep(0.1)
        self.gain_spin.setRange(0.0, 24.0)
        self.gain_spin.setValue(1.0)
        self.gain_spin.valueChanged.connect(self.on_gain_changed)
        form.addRow("Gain:", self.gain_spin)

        self.update_interval_spin = QDoubleSpinBox()
        self.update_interval_spin.setDecimals(2)
        self.update_interval_spin.setSingleStep(0.25)
        self.update_interval_spin.setRange(0.1, 60.0)
        self.update_interval_spin.setValue(1.0)
        self.update_interval_spin.setSuffix(" s")
        self.update_interval_spin.valueChanged.connect(self.on_interval_changed)
        form.addRow("Update interval:", self.update_interval_spin)

        self.scale_checkbox = QCheckBox("Use micron scale")
        self.scale_checkbox.toggled.connect(self.on_scale_toggled)
        form.addRow("Scale:", self.scale_checkbox)

        self.scale_spin = QDoubleSpinBox()
        self.scale_spin.setDecimals(3)
        self.scale_spin.setSingleStep(0.1)
        self.scale_spin.setRange(0.000001, 1_000_000.0)
        self.scale_spin.setValue(5.0)
        self.scale_spin.setSuffix(" µm/pixel")
        self.scale_spin.valueChanged.connect(self.on_scale_value_changed)
        form.addRow("Microns/pixel:", self.scale_spin)

        grid.addLayout(form, 1, 0, 1, 3)
        root.addWidget(controls)

        self.status = QLabel("Ready")
        root.addWidget(self.status)
        self.setCentralWidget(central)
        self.resize(1200, 800)

    def _open_camera(self) -> None:
        try:
            self.camera.open()
            lo, hi = self.camera.gain_limits()
            self.gain_spin.setRange(lo, hi)
            self.gain_spin.setValue(clamp_float(1.0, lo, hi))
            elo, ehi = self.camera.exposure_limits()
            self.exposure_spin.setRange(max(1.0, elo), ehi)
            self.camera.set_exposure_us(self.exposure_spin.value())
            self.camera.set_gain(self.gain_spin.value())
            self.camera.init_software_trigger()
            self.camera.start_acquisition()
            self.status.setText("Camera open; software trigger acquisition ready")
        except Exception as exc:
            self.status.setText(f"Camera open failed: {exc}")
            QMessageBox.critical(self, "Camera Error", str(exc))

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.timer.stop()
        self.camera.close()
        event.accept()

    def on_start_stop(self, checked: bool) -> None:
        if checked:
            self.timer.start()
            self.status.setText("Timer acquisition running")
            self.on_timer()
        else:
            self.timer.stop()
            self.status.setText("Timer acquisition stopped")

    def on_timer(self) -> None:
        if self.acquiring_image:
            return
        self.acquiring_image = True
        try:
            image = self.camera.make_image(timeout_ms=5000)
            self.on_frame(image)
            self.status.setText("Last image acquired")
        except ids_peak.TimeoutException:
            self.status.setText("Timed out waiting for triggered image")
        except Exception as exc:
            self.status.setText(f"Acquisition error: {exc}")
            self.start_stop.blockSignals(True)
            self.start_stop.setChecked(False)
            self.start_stop.blockSignals(False)
            self.timer.stop()
            QMessageBox.warning(self, "Camera Error", str(exc))
        finally:
            self.acquiring_image = False

    def on_frame(self, frame: object) -> None:
        image = np.asarray(frame)
        self.current_image = image
        self.image_widget.set_image(image)
        self.update_slices()

    def on_exposure_changed(self, value: float) -> None:
        try:
            self.camera.set_exposure_us(value)
        except Exception as exc:
            self.status.setText(f"Exposure set failed: {exc}")

    def on_gain_changed(self, value: float) -> None:
        try:
            self.camera.set_gain(value)
        except Exception as exc:
            self.status.setText(f"Gain set failed: {exc}")

    def on_interval_changed(self, value: float) -> None:
        self.timer.setInterval(int(value * 1000))

    def capture_single_window(self) -> None:
        try:
            image = self.camera.acquire_single(
                exposure_us=self.exposure_spin.value(),
                gain=self.gain_spin.value(),
                timeout_ms=5000,
            )
            win = CaptureWindow(image, self)
            self.capture_windows.append(win)
            win.show()
        except Exception as exc:
            self.status.setText(f"Capture failed: {exc}")
            QMessageBox.warning(self, "Capture Error", str(exc))

    def on_cursor_changed(self, x: int, y: int) -> None:
        self.update_position_label()
        self.update_slices()

    def on_origin_changed(self, x0: int, y0: int) -> None:
        self.update_position_label()
        self.update_slices()

    def on_scale_toggled(self, checked: bool) -> None:
        self.scale_enabled = checked
        self.update_position_label()
        self.update_slices()

    def on_scale_value_changed(self, value: float) -> None:
        self.microns_per_pixel = value
        self.update_position_label()
        self.update_slices()

    def displayed_xy(self) -> Tuple[float, float, str]:
        dx = self.image_widget.cursor_x - self.image_widget.origin_x
        dy = self.image_widget.cursor_y - self.image_widget.origin_y
        if self.scale_enabled:
            return dx * self.microns_per_pixel, dy * self.microns_per_pixel, "µm"
        return float(dx), float(dy), "px"

    def update_position_label(self) -> None:
        x, y, units = self.displayed_xy()
        self.position_label.setText(
            f"Cursor: X={x:.3f} {units}, Y={y:.3f} {units}   "
            f"Origin: X0={self.image_widget.origin_x} px, Y0={self.image_widget.origin_y} px"
        )

    def update_slices(self) -> None:
        image = self.current_image
        if image is None or image.ndim != 2:
            return
        h, w = image.shape
        x = clamp(self.image_widget.cursor_x, 0, w - 1)
        y = clamp(self.image_widget.cursor_y, 0, h - 1)
        horizontal = image[y, :]
        vertical = image[:, x]
        hx = np.arange(w, dtype=float) - self.image_widget.origin_x
        vy = np.arange(h, dtype=float) - self.image_widget.origin_y
        units = "pixels"
        if self.scale_enabled:
            hx *= self.microns_per_pixel
            vy *= self.microns_per_pixel
            units = "µm"
        self.h_curve.setData(hx, horizontal)
        self.v_curve.setData(vy, vertical)
        self.h_plot.setLabel("bottom", "X", units=units)
        self.v_plot.setLabel("bottom", "Y", units=units)


# ----------------------- Python-callable single image API --------------------


def acquire_single_image(exposure_us: float = 5000.0, gain: float = 1.0) -> np.ndarray:
    """Acquire one IDS camera image and return a 2-D NumPy integer array."""
    cam = IDSPeakMonoCamera()
    try:
        cam.open()
        image = cam.acquire_single(exposure_us=exposure_us, gain=gain, timeout_ms=5000)
        if not np.issubdtype(image.dtype, np.integer):
            image = image.astype(np.uint16)
        if image.ndim != 2:
            image = np.squeeze(image)
        if image.ndim != 2:
            raise RuntimeError(f"Expected 2-D image, got shape {image.shape}")
        return np.array(image, copy=True)
    finally:
        cam.close()


# ---------------------------------- main -------------------------------------


def main() -> int:
    app = QApplication(sys.argv)
    pg.setConfigOptions(imageAxisOrder="row-major")
    window = USBCameraControlWidget()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
