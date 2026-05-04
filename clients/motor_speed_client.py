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
    QDoubleSpinBox,
)

I2C_BUS = 1              # Raspberry Pi 4 default I2C bus
I2C_ADDR = 0x64          # Seeeduino slave address: decimal 100

REG_PWM_LOW = 1
REG_PWM_HIGH = 2
REG_EDGE_TIME_LOW = 3    # 3-byte little-endian microseconds/revolution

CAL_FILE = Path(__file__).with_name("motorcal.txt")
MAX_SETPOINT_HZ = 164.0

def load_motor_calibration(path: Path):
    speeds = []
    registers = []

    with path.open("r") as f:
        tokens = f.read().split()

    # Skip header tokens / comments by parsing only numeric pairs
    numeric = []
    for token in tokens:
        try:
            numeric.append(float(token))
        except ValueError:
            pass

    if len(numeric) < 4 or len(numeric) % 2 != 0:
        raise ValueError(f"Invalid calibration file: {path}")

    for i in range(0, len(numeric), 2):
        speeds.append(numeric[i])
        registers.append(numeric[i + 1])

    # np.interp requires x-values sorted ascending.
    pairs = sorted(zip(speeds, registers), key=lambda p: p[0])

    # Remove duplicate speed entries, keeping the lowest register value.
    # This matters because motorcal.txt has multiple 0.00 Hz entries.
    unique_speeds = []
    unique_registers = []

    for speed, reg in pairs:
        if unique_speeds and abs(speed - unique_speeds[-1]) < 1e-9:
            unique_registers[-1] = min(unique_registers[-1], reg)
        else:
            unique_speeds.append(speed)
            unique_registers.append(reg)

    return np.array(unique_speeds), np.array(unique_registers)


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

        self.setpoint_label = QLabel("Spindle speed setpoint")
        self.setpoint_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.speed_setpoint = QDoubleSpinBox()
        self.speed_setpoint.setRange(0.0, MAX_SETPOINT_HZ)
        self.speed_setpoint.setDecimals(2)
        self.speed_setpoint.setSingleStep(1.0)
        self.speed_setpoint.setSuffix(" Hz")
        self.speed_setpoint.setValue(0.0)
        self.speed_setpoint.valueChanged.connect(self.on_setpoint_changed)

        layout = QVBoxLayout()
        layout.addWidget(self.speed_dial)
        layout.addWidget(self.speed_label)
        layout.addWidget(self.setpoint_label)
        layout.addWidget(self.speed_setpoint)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_speed)
        self.timer.start(1000)

        self.cal_speeds_hz, self.cal_registers = load_motor_calibration(CAL_FILE)

        self.on_setpoint_changed(0)

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

    def on_setpoint_changed(self, speed_hz: float):
        pwm_value = int(round(np.interp(
            speed_hz,
            self.cal_speeds_hz,
            self.cal_registers
        )))
    
        pwm_value = max(0, min(0xFFFF, pwm_value))
    
        low = pwm_value & 0xFF
        high = (pwm_value >> 8) & 0xFF
    
        self.setpoint_label.setText(
            f"Spindle speed setpoint: {speed_hz:.2f} Hz → register {pwm_value}"
        )
    
        try:
            self.bus.write_byte_data(I2C_ADDR, REG_PWM_LOW, low)
            self.bus.write_byte_data(I2C_ADDR, REG_PWM_HIGH, high)
    
        except OSError as e:
            self.setpoint_label.setText(f"I2C write error: {e}")


    def closeEvent(self, event):
        self.bus.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorSpeedClient()
    window.resize(420, 320)
    window.show()
    sys.exit(app.exec())
