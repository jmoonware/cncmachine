#!/usr/bin/env python3
"""
PyQt6 IDS peak USB monochrome camera control widget.

Target: Raspberry Pi 4 + IDS 3680XLE-M monochrome USB camera.

Dependencies:
    p([github.com](https://github.com/ids-imaging/ids-peak-examples/blob/main/python))umpy pillow ids-peak ids-peak-common ids-peak-icv

pip install PyQt6 pyqtgraph numpy pillow ids-peak ids-peak-common ids-peak-icv

IDS peak runtime / GenTL producer must also be installed on the system.

Run:
    python ids_usb_camera_pyqt6_widget.py

Notes:
    * This uses the IDS peak generic SDK style: DeviceManager -> Device -> DataStream
      -> RemoteDevice NodeMap -> WaitForFinishedBuffer.
    * It attempts to set Mono8 if available. If the camera outputs >8-bit mono data,
      the displayed image is contrast-scaled to 8-bit while the returned/captured NumPy
      array preserves the integer depth provided by the SDK conversion path.
    * ExposureTime is in microseconds for standard GenICam IDS nodes.
"""

from __future__ import annotations

import os
import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, Any, cast

import numpy as np

from PyQt6.QtCore import Qt, pyqtSignal, QObject, QPointF
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QAction
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
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from ids_peak import ids_peak

try:
    from ids_peak_common import PixelFormat
    import ids_peak_icv
    from ids_peak_icv.pipeline import DefaultPipeline
except Exception:  # The app can still fail clearly at runtime if unavailable.
    PixelFormat = None
    ids_peak_icv = None
    DefaultPipeline = None


# ------------------------------- Utilities ---------------------------------


def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def normalize_to_u8(image: np.ndarray) -> np.ndarray:
    """Return an 8-bit view suitable for display, preserving contrast."""
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
    """Create an owned QImage from a 2-D NumPy image."""
    img8 = normalize_to_u8(image)
    h, w = img8.shape
    qimg = QImage(img8.data, w, h, img8.strides[0], QImage.Format.Format_Grayscale8)
    return qimg.copy()


def save_numpy_image(path: str, image: np.ndarray) -> None:
    """Save image as PNG/TIFF-compatible integer image using Pillow when possible."""
    try:
        from PIL import Image

        arr = np.ascontiguousarray(image)
        if arr.dtype == np.uint8:
            im = Image.fromarray(arr, mode="L")
        elif arr.dtype == np.uint16:
            im = Image.fromarray(arr, mode="I;16")
        else:
            # PNG can store 8/16-bit grayscale. Scale unusual integer types to 16-bit.
            arr16 = arr.astype(np.uint16, copy=False)
            im = Image.fromarray(arr16, mode="I;16")
        im.save(path)
    except ImportError:
        # Fallback: use Qt 8-bit save if Pillow is not installed.
        numpy_gray_to_qimage(image).save(path)


def node_available(nodemap: Any, name: str) -> bool:
    try:
        nodemap.FindNode(name)
        return True
    except Exception:
        return False


def set_enum_if_available(nodemap: Any, node_name: str, entry_name: str) -> bool:
    try:
        node = cast(ids_peak.EnumerationNode, nodemap.FindNode(node_name))
        node.SetCurrentEntry(entry_name)
        return True
    except Exception:
        return False


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
    """Small IDS peak wrapper for a single IDS monochrome camera."""

    def __init__(self) -> None:
        self.device = None
        self.nodemap = None
        self.datastream = None
        self.pipeline = None
        self._opened = False
        self._streaming = False
        self._lock = threading.RLock()

    def open(self) -> None:
        with self._lock:
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

            # Load default user set where available.
            try:
                cast(ids_peak.EnumerationNode, self.nodemap.FindNode("UserSetSelector")).SetCurrentEntry("Default")
                user_set_load = cast(ids_peak.CommandNode, self.nodemap.FindNode("UserSetLoad"))
                user_set_load.Execute()
                user_set_load.WaitUntilDone()
            except Exception:
                pass

            # Prefer a monochrome pixel format.
            # IDS/GenICam cameras commonly expose Mono8, Mono10, Mono12, Mono16, etc.
            for fmt in ("Mono8", "Mono10", "Mono12", "Mono16"):
                if set_enum_if_available(self.nodemap, "PixelFormat", fmt):
                    break

            if DefaultPipeline is not None and PixelFormat is not None:
                self.pipeline = DefaultPipeline()
                # Use MONO_8 output when available. Different ids_peak_common versions
                # have slightly different enum names, so try common spellings.
                for attr in ("MONO_8", "Mono8", "Mono_8"):
                    if hasattr(PixelFormat, attr):
                        self.pipeline.output_pixel_format = getattr(PixelFormat, attr)
                        break

            self._allocate_buffers()
            self._opened = True

    def _allocate_buffers(self) -> None:
        if self.nodemap is None or self.datastream is None:
            raise RuntimeError("Camera not open")
        payload_size = cast(ids_peak.IntegerNode, self.nodemap.FindNode("PayloadSize")).Value()
        buffer_count = self.datastream.NumBuffersAnnouncedMinRequired()
        for _ in range(buffer_count):
            buffer = self.datastream.AllocAndAnnounceBuffer(payload_size)
            self.datastream.QueueBuffer(buffer)

    def close(self) -> None:
        with self._lock:
            try:
                self.stop_streaming()
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
            self.pipeline = None

            if self._opened:
                try:
                    ids_peak.Library.Close()
                except Exception:
                    pass
            self._opened = False

    def set_exposure_us(self, exposure_us: float) -> None:
        if self.nodemap is None:
            return
        with self._lock:
            set_float_node(self.nodemap, "ExposureTime", exposure_us)

    def set_gain(self, gain: float) -> None:
        if self.nodemap is None:
            return
        with self._lock:
            set_float_node(self.nodemap, "Gain", gain)

    def gain_limits(self) -> Tuple[float, float]:
        if self.nodemap is None:
            return 0.0, 24.0
        return get_float_node_limits(self.nodemap, "Gain", (0.0, 24.0))

    def exposure_limits(self) -> Tuple[float, float]:
        if self.nodemap is None:
            return 1.0, 10_000_000.0
        return get_float_node_limits(self.nodemap, "ExposureTime", (1.0, 10_000_000.0))

    def start_streaming(self) -> None:
        with self._lock:
            if self._streaming:
                return
            if self.nodemap is None or self.datastream is None:
                raise RuntimeError("Camera not open")
            try:
                cast(ids_peak.IntegerNode, self.nodemap.FindNode("TLParamsLocked")).SetValue(1)
            except Exception:
                pass
            self.datastream.StartAcquisition()
            cmd = cast(ids_peak.CommandNode, self.nodemap.FindNode("AcquisitionStart"))
            cmd.Execute()
            cmd.WaitUntilDone()
            self._streaming = True

    def stop_streaming(self) -> None:
        with self._lock:
            if not self._streaming:
                return
            if self.nodemap is None or self.datastream is None:
                return
            try:
                cast(ids_peak.CommandNode, self.nodemap.FindNode("AcquisitionStop")).Execute()
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
            self._streaming = False

    def acquire_frame(self, timeout_ms: int = 2000) -> np.ndarray:
        """Acquire one frame. Caller must have started acquisition."""
        if self.datastream is None:
            raise RuntimeError("Camera not open")

        buffer = self.datastream.WaitForFinishedBuffer(ids_peak.Timeout(timeout_ms))
        try:
            image_view = buffer.ToImageView()
            if self.pipeline is not None:
                converted = self.pipeline.process(image_view)
                arr = converted.to_numpy_array()
            else:
                arr = image_view.to_numpy_array()
            arr = np.asarray(arr)
            if arr.ndim == 3:
                arr = arr[:, :, 0]
            return np.array(arr, copy=True)
        finally:
            self.datastream.QueueBuffer(buffer)

    def acquire_single(self, exposure_us: float = 5000.0, gain: float = 1.0, timeout_ms: int = 5000) -> np.ndarray:
        """Set exposure/gain and acquire a single frame, preserving previous stream state."""
        with self._lock:
            was_streaming = self._streaming
            if not was_streaming:
                self.start_streaming()
            self.set_exposure_us(exposure_us)
            self.set_gain(gain)
            # Allow settings to settle for at least one exposure interval.
            time.sleep(max(0.002, exposure_us / 1_000_000.0))
            frame = self.acquire_frame(timeout_ms=timeout_ms)
            if not was_streaming:
                self.stop_streaming()
            return frame


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
            pen = QPen(QColor(255, 60, 60), 1)
            painter.setPen(pen)
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
        self.image_label.setScaledContents(False)
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


class AcquisitionSignals(QObject):
    frame = pyqtSignal(object)
    error = pyqtSignal(str)


class USBCameraControlWidget(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IDS 3680XLE-M USB Camera Control")
        self.camera = IDSPeakMonoCamera()
        self.signals = AcquisitionSignals()
        self.signals.frame.connect(self.on_frame)
        self.signals.error.connect(self.on_error)

        self.worker_thread: Optional[threading.Thread] = None
        self.worker_stop = threading.Event()
        self.current_image: Optional[np.ndarray] = None
        self.capture_windows: list[CaptureWindow] = []

        self.scale_enabled = False
        self.microns_per_pixel = 5.0

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

        self.start_stop = QCheckBox("Start continuous streaming")
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
            self.status.setText("Camera open")
        except Exception as exc:
            self.status.setText(f"Camera open failed: {exc}")
            QMessageBox.critical(self, "Camera Error", str(exc))

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.stop_worker()
        self.camera.close()
        event.accept()

    def on_start_stop(self, checked: bool) -> None:
        if checked:
            self.start_worker()
        else:
            self.stop_worker()

    def start_worker(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            return
        self.worker_stop.clear()
        try:
            self.camera.set_exposure_us(self.exposure_spin.value())
            self.camera.set_gain(self.gain_spin.value())
            self.camera.start_streaming()
        except Exception as exc:
            self.start_stop.blockSignals(True)
            self.start_stop.setChecked(False)
            self.start_stop.blockSignals(False)
            self.on_error(str(exc))
            return

        self.worker_thread = threading.Thread(target=self._stream_worker, daemon=True)
        self.worker_thread.start()
        self.status.setText("Streaming")

    def stop_worker(self) -> None:
        self.worker_stop.set()
        try:
            self.camera.stop_streaming()
        except Exception:
            pass
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        self.worker_thread = None
        self.status.setText("Stopped")

    def _stream_worker(self) -> None:
        while not self.worker_stop.is_set():
            try:
                frame = self.camera.acquire_frame(timeout_ms=1000)
                self.signals.frame.emit(frame)
            except ids_peak.TimeoutException:
                continue
            except ids_peak.AbortedException:
                break
            except Exception as exc:
                self.signals.error.emit(str(exc))
                break

    def on_frame(self, frame: object) -> None:
        image = np.asarray(frame)
        self.current_image = image
        self.image_widget.set_image(image)
        self.update_slices()

    def on_error(self, text: str) -> None:
        self.status.setText(f"Error: {text}")
        QMessageBox.warning(self, "Camera Error", text)

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

    def capture_single_window(self) -> None:
        try:
            was_checked = self.start_stop.isChecked()
            if was_checked:
                # Use the newest live frame for snappy captures while streaming.
                image = np.array(self.current_image, copy=True) if self.current_image is not None else None
                if image is None:
                    image = self.camera.acquire_frame(timeout_ms=2000)
            else:
                image = self.camera.acquire_single(
                    exposure_us=self.exposure_spin.value(),
                    gain=self.gain_spin.value(),
                    timeout_ms=5000,
                )
            win = CaptureWindow(image, self)
            self.capture_windows.append(win)
            win.show()
        except Exception as exc:
            self.on_error(f"Capture failed: {exc}")

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


def clamp_float(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ----------------------- Python-callable single image API --------------------


def acquire_single_image(exposure_us: float = 5000.0, gain: float = 1.0) -> np.ndarray:
    """
    Acquire one IDS camera image and return a 2-D NumPy integer array.

    This function is intentionally GUI-independent so it can be imported from
    measurement scripts:

        from ids_usb_camera_pyqt6_widget import acquire_single_image
        img = acquire_single_image(exposure_us=10000, gain=2.5)

    Returns:
        np.ndarray with shape (height, width), integer dtype.
    """
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

