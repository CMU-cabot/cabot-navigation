#!/usr/bin/env python3

# Copyright (c) 2025  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import can
import struct
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, QHBoxLayout, QCheckBox
from python_qt_binding.QtCore import Qt, QTimer  # Add QTimer for periodic tasks


class Battery(Plugin):
    def __init__(self, context):
        super(Battery, self).__init__(context)
        self.setObjectName("BatterySimulationPlugin")

        # Create QWidget
        self._widget = QWidget()
        self._widget.setWindowTitle("Battery Simulation")

        # Initialize UI components
        self.init_ui()

        # Add widget to the user interface
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + f" ({context.serial_number()})")
        context.add_widget(self._widget)

    def init_ui(self):
        # Main layout
        main_layout = QVBoxLayout()

        # Create horizontal layout for batteries
        batteries_layout = QHBoxLayout()

        # Create sliders and checkboxes for 4 batteries
        self.battery_sliders = []
        self.battery_checkboxes = []
        self.battery_serials = ["0x1A2B", "0x1A2C", "0x1A2D", "0x1A2E"]  # Fixed serial numbers
        for i in range(4):
            battery_layout = QVBoxLayout()
            battery_layout.setContentsMargins(10, 10, 10, 10)  # Add padding to battery panels
            battery_label = QLabel(f"Battery {i + 1} (Serial: {self.battery_serials[i]})")
            battery_layout.addWidget(battery_label)

            # Add checkbox
            checkbox = QCheckBox("Enable")
            checkbox.setChecked(True)
            self.battery_checkboxes.append(checkbox)
            battery_layout.addWidget(checkbox)

            voltage_slider, voltage_label = self.create_slider(
                "Voltage (V):", 250, 300, 1, 270, battery_layout, scale=0.1
            )
            current_slider, current_label = self.create_slider(
                "Current (mA):", -2000, 2000, 100, 0, battery_layout
            )
            percentage_slider, percentage_label = self.create_slider(
                "Percentage (%):", 0, 100, 5, 50, battery_layout
            )
            temperature_slider, temperature_label = self.create_slider(
                "Temperature (°C):", 15, 40, 1, 25, battery_layout
            )

            self.battery_sliders.append({
                "voltage": voltage_slider,
                "current": current_slider,
                "percentage": percentage_slider,
                "temperature": temperature_slider
            })

            batteries_layout.addLayout(battery_layout)

        # Add batteries layout to the main layout
        main_layout.addLayout(batteries_layout)

        # Add auto-send checkbox
        self.auto_send_checkbox = QCheckBox("Auto Send Every Second")
        main_layout.addWidget(self.auto_send_checkbox)

        # Send button
        send_button = QPushButton("Send Data")
        send_button.clicked.connect(self.send_data)
        main_layout.addWidget(send_button)

        # Set up a timer for auto-send
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_data)
        self.auto_send_checkbox.stateChanged.connect(self.toggle_auto_send)

        # Set layout to the widget
        self._widget.setLayout(main_layout)

    def toggle_auto_send(self, state):
        """Enable or disable auto-send based on the checkbox state."""
        if state == Qt.Checked:
            self.timer.start(1000)  # Send data every 1000 ms (1 second)
        else:
            self.timer.stop()

    def create_slider(self, label_text, min_val, max_val, step, default, layout, scale=1):
        """Helper to create labeled sliders."""
        layout_row = QVBoxLayout()
        label = QLabel(f"{label_text} {default * scale:.1f}")
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setSingleStep(step)
        slider.setValue(default)
        slider.valueChanged.connect(lambda value: label.setText(f"{label_text} {value * scale:.1f}" if scale != 1 else f"{label_text} {value}"))
        layout_row.addWidget(label)
        layout_row.addWidget(slider)
        layout.addLayout(layout_row)
        return slider, label

    def send_data(self):
        """Send CAN messages for all batteries."""
        try:
            bus = can.interface.Bus(channel='can0', bustype='socketcan')

            # Send individual battery data
            for i, sliders in enumerate(self.battery_sliders):
                if self.battery_checkboxes[i].isChecked():
                    # Get slider values
                    voltage = sliders["voltage"].value() * 100  # Convert to mV
                    current = sliders["current"].value()       # mA
                    percentage = sliders["percentage"].value()  # %
                    temperature = int(sliders["temperature"].value() * 10 + 2731.5)  # Convert to deci-degrees
                else:
                    # Send all values as 0xFFFF if unchecked
                    voltage = 0xFFFF
                    current = 0x7FFF
                    percentage = 0xFFFF
                    temperature = 0xFFFF

                # Prepare CAN message
                arbitration_id = 0x518 + i
                msg = can.Message(
                    arbitration_id=arbitration_id,
                    data=struct.pack("<HhHH", voltage, current, percentage, temperature),
                    is_extended_id=False
                )

                # Send CAN message
                bus.send(msg)
                print(f"Sent message for Battery {i + 1} (Serial: {self.battery_serials[i]}): {msg}")

            # Send serial numbers of all batteries in a single message
            serial_numbers = [int(serial, 16) for serial in self.battery_serials]  # Convert hex strings to integers
            msg = can.Message(
                arbitration_id=0x520,
                data=struct.pack("<HHHH", *serial_numbers),  # Pack all 4 serial numbers
                is_extended_id=False
            )

            # Send CAN message
            bus.send(msg)
            print(f"Sent serial numbers: {self.battery_serials}")

        except can.CanError as e:
            print(f"CAN error occurred: {e}")

    def add_arguments(parser):
        pass
