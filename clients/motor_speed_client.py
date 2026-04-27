#!/usr/bin/env python3

import sys
from smbus2 import SMBus, i2c_msg
import numpy as np
import time

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QLabel,
    QDial,
    QSlider,
)

I2C_BUS = 1              # Raspberry Pi 4 default I2C bus
I2C_ADDR = 0x64          # Seeeduino slave address: decimal 100

REG_PWM_LOW = 1
REG_PWM_HIGH = 2
REG_EDGE_TIME_LOW = 3    # 3-byte little-endian microseconds/revolution


class MotorSpeedClient(QWidget):
    def __init__(self):
        super().__init__()

        self.bus = SMBus(I2C_BUS)

        self.setWindowTitle("Motor Speed Control")

        self.speed_dial = QDial()
        self.speed_dial.setRange(0, 100)
        self.speed_dial.setNotchesVisible(True)
        self.speed_dial.setEnabled(False)

        self.speed_label = QLabel("0.00 Hz")
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_label.setStyleSheet("font-size: 32px; font-weight: bold;")

        self.slider_label = QLabel("Motor command: 0%")
        self.slider_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(0)
        self.speed_slider.valueChanged.connect(self.on_slider_changed)

        layout = QVBoxLayout()
        layout.addWidget(self.speed_dial)
        layout.addWidget(self.speed_label)
        layout.addWidget(self.slider_label)
        layout.addWidget(self.speed_slider)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_speed)
        self.timer.start(1000)

        self.on_slider_changed(0)

    def read_edge_time_us(self) -> int:
        # Write register address 3, then read 3 bytes.
        write = i2c_msg.write(I2C_ADDR, [REG_EDGE_TIME_LOW])
        read = i2c_msg.read(I2C_ADDR, 3)
        self.bus.i2c_rdwr(write, read)

        b = list(read)
        return b[0] | (b[1] << 8) | (b[2] << 16)

    def update_speed(self):
        try:
            samples = []
            for _ in range(3):
                samples.append(self.read_edge_time_us())
                time.sleep(0.25)

            elapsed_us = np.median(samples)

            if elapsed_us == 0:
                hz = 0.0
            else:
                hz = 1_000_000.0 / elapsed_us
                if hz > 200.:
                    return # don't update value

            self.speed_label.setText(f"{hz:.2f} Hz")

            # Dial display range: 0–100 Hz, clipped.
            self.speed_dial.setValue(max(0, min(100, round(hz))))

        except OSError as e:
            self.speed_label.setText(f"I2C error: {e}")

    def on_slider_changed(self, percent: int):
        self.slider_label.setText(f"Motor command: {percent}%")

        # 0% -> 0x2300
        # 100% -> 0x0000
        pwm_value = round(0x2300 * (100 - percent) / 100)

        low = pwm_value & 0xFF
        high = (pwm_value >> 8) & 0xFF

        try:
            # Remote firmware expects single-byte register + single-byte value.
            self.bus.write_byte_data(I2C_ADDR, REG_PWM_LOW, low)
            self.bus.write_byte_data(I2C_ADDR, REG_PWM_HIGH, high)

        except OSError as e:
            self.slider_label.setText(f"I2C write error: {e}")

    def closeEvent(self, event):
        self.bus.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorSpeedClient()
    window.resize(420, 320)
    window.show()
    sys.exit(app.exec())
