#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

import math

from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin

from python_qt_binding.QtWidgets import QVBoxLayout, QPushButton, QGridLayout, QLabel
from std_msgs.msg import Bool, Int8, UInt8, Int16
from python_qt_binding.QtCore import Qt, QEvent, QTimer
from python_qt_binding.QtWidgets import QGraphicsView, QGraphicsScene
from python_qt_binding.QtGui import QPen


class Handle(Plugin):

    def __init__(self, context):
        super(Handle, self).__init__(context)
        self.setObjectName('Handle')

        self.servo_free = True
        self.servo_pos = 0
        self.servo_target_msg = None
        self._context = context
        self._node = context.node
        self._logger = self._node.get_logger()
        self._widget = QWidget()
        self._widget.setWindowTitle("CaBot Handle")
        context.add_widget(self._widget)

        # Add buttons
        layout = QGridLayout()
        layout.setSpacing(20)
        layout.setAlignment(Qt.AlignCenter)

        self._widget.setLayout(layout)

        def create_button(label, shortcut, width=60, height=60):
            button = QPushButton()
            button.setFixedSize(width, height)
            layout = QVBoxLayout()
            layout.addWidget(QLabel(label), alignment=Qt.AlignCenter)
            layout.addWidget(QLabel(f'({shortcut})'), alignment=Qt.AlignCenter)
            button.setLayout(layout)
            return button
        self.buttons = [
            create_button('Up', 'i'),
            create_button('Down', ','),
            create_button('Left', 'j'),
            create_button('Right', 'l'),
            create_button('Center', 'k')
        ]

        layout.addWidget(self.buttons[0], 0, 1, alignment=Qt.AlignCenter)  # Up
        layout.addWidget(self.buttons[2], 1, 0, alignment=Qt.AlignCenter)  # Left
        layout.addWidget(self.buttons[4], 1, 1, alignment=Qt.AlignCenter)  # Center
        layout.addWidget(self.buttons[3], 1, 2, alignment=Qt.AlignCenter)  # Right
        layout.addWidget(self.buttons[1], 2, 1, alignment=Qt.AlignCenter)  # Down

        # Directional Indicator
        # Get the use_directional_indicator parameter
        self._node.declare_parameter('use_directional_indicator', False)
        self.use_directional_indicator = self._node.get_parameter('use_directional_indicator').get_parameter_value().bool_value
        self._logger.info(f'use_directional_indicator: {self.use_directional_indicator}')

        self.canvas = QGraphicsView()
        self.canvas.setFixedSize(150, 150)
        self.scene = QGraphicsScene()
        self.canvas.setScene(self.scene)
        # Draw a circle in the canvas
        self.circle = self.scene.addEllipse(0, 0, 100, 100)
        # Draw a line from the center of the circle toward the top of the circle
        self.line = self.scene.addLine(50, 50, 50, 0)
        layout.addWidget(self.canvas, 3, 1, alignment=Qt.AlignCenter)

        # Add label to show the status of servo_free
        self.servo_free_label = QLabel('Free: True')
        layout.addWidget(self.servo_free_label, 3, 2, alignment=Qt.AlignCenter)

        # timer to draw the indicator
        def update_plot():
            pen = QPen(Qt.lightGray if self.servo_free else Qt.black, 5)
            if self.servo_target_msg and not self.servo_free:
                self.servo_pos = self.servo_pos * 0.8 + self.servo_target_msg.data * 0.2
                if abs(self.servo_pos - self.servo_target_msg.data) < 0.1:
                    self.servo_pos = self.servo_target_msg.data
                    self.servo_target_msg = None
                self._logger.info(f'{self.servo_pos=}')

            length = 50
            x = 50 + length * math.sin(math.radians(self.servo_pos))
            y = 50 - length * math.cos(math.radians(self.servo_pos))
            self.circle.setPen(pen)
            self.line.setLine(50, 50, x, y)
            self.line.setPen(pen)

        self._update_plot_timer = QTimer(self._widget)
        self._update_plot_timer.setInterval(20)
        self._update_plot_timer.timeout.connect(update_plot)
        self._update_plot_timer.start()

        if not self.use_directional_indicator:
            self.canvas.hide()

        # Add touch button
        self.touch_button = create_button('Touch', 't, toggle', 120, 180)
        layout.addWidget(self.touch_button, 5, 0, 1, 3, alignment=Qt.AlignCenter)

        # Add labels to show vibration status
        def create_label(label):
            label = QLabel(label)
            label.setFixedSize(80, 40)
            label.setStyleSheet('background-color: none; padding: 10px;')
            return label
        # Add labels for vibrator values
        self.vibrator_labels = [
            create_label('Vib 1: 0'),
            create_label('Vib 2: 0'),
            create_label('Vib 3: 0'),
            create_label('Vib 4: 0')
        ]

        layout.addWidget(self.vibrator_labels[0], 4, 1, alignment=Qt.AlignCenter)  # Up
        layout.addWidget(self.vibrator_labels[1], 6, 1, alignment=Qt.AlignCenter)  # Down
        layout.addWidget(self.vibrator_labels[2], 5, 0, alignment=Qt.AlignCenter)  # Left
        layout.addWidget(self.vibrator_labels[3], 5, 2, alignment=Qt.AlignCenter)  # Right

        self.vibrator_labels[1].hide()  # Hide the Down vibrator always
        if self.use_directional_indicator:
            self.vibrator_labels[2].hide()
            self.vibrator_labels[3].hide()

        # Set focus to touch button at launch
        self.touch_button.setFocus()

        # Publisher for the topics
        self.publisher_pushed = self._node.create_publisher(Int8, 'pushed', 10)
        self.publisher_touch = self._node.create_publisher(Int16, 'touch', 10)
        self.publisher_servo = self._node.create_publisher(Int16, 'servo_pos', 10)
        self.timer = self._node.create_timer(0.02, self.publish_topics)

        # Subscribers for the vibrator topics
        self.subscribers = [
            self._node.create_subscription(UInt8, 'vibrator1', self.vibrator_callback(0), 10),
            self._node.create_subscription(UInt8, 'vibrator2', self.vibrator_callback(1), 10),
            self._node.create_subscription(UInt8, 'vibrator3', self.vibrator_callback(2), 10),
            self._node.create_subscription(UInt8, 'vibrator4', self.vibrator_callback(3), 10),
            self._node.create_subscription(Bool, 'servo_free', self.servo_free_callback, 10),
            self._node.create_subscription(Int16, 'servo_target', self.servo_target_callback, 10),
        ]

        # Install event filter to capture key presses
        self._widget.installEventFilter(self)

    def publish_topics(self):
        msg_pushed = Int8()
        msg_pushed.data = 0  # Default value indicating no button pressed
        for i, button in enumerate(self.buttons):
            if button.isDown():
                msg_pushed.data |= (1 << i)
        self.publisher_pushed.publish(msg_pushed)

        msg_touch = Int16()
        msg_touch.data = 1 if self.touch_button.isDown() else 0
        self.publisher_touch.publish(msg_touch)

        msg_servo = Int16()
        msg_servo.data = int(self.servo_pos)
        self.publisher_servo.publish(msg_servo)

    def vibrator_callback(self, index):
        def inner_function(msg):
            self.vibrator_labels[index].setText(f'Vib {index + 1}: {msg.data}')
            self.vibrator_labels[index].setStyleSheet('background-color: red; padding: 10px;')

            def callback():
                self.vibrator_timer.cancel()
                self.vibrator_labels[index].setStyleSheet('background-color: none; padding: 10px;')
            self.vibrator_timer = self._node.create_timer((10 * msg.data) / 1000.0, callback)

            def callback2():
                self.vibrator_timer2.cancel()
                self.vibrator_labels[index].setText(f'Vib {index + 1}: 0')
            self.vibrator_timer2 = self._node.create_timer(2.0, callback2)
        return inner_function

    def servo_free_callback(self, msg):
        self.servo_free = msg.data
        self.servo_free_label.setText(f'Free: {self.servo_free}')

    def servo_target_callback(self, msg):
        self.servo_target_msg = msg

    def eventFilter(self, obj, event):
        # self._logger.info(f'eventFilter: {event=}')
        if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
            if event.key() == Qt.Key_I:
                self.buttons[0].setDown(True)
            elif event.key() == Qt.Key_Comma:
                self.buttons[1].setDown(True)
            elif event.key() == Qt.Key_J:
                self.buttons[2].setDown(True)
            elif event.key() == Qt.Key_L:
                self.buttons[3].setDown(True)
            elif event.key() == Qt.Key_K:
                self.buttons[4].setDown(True)
            elif event.key() == Qt.Key_T:
                self.touch_button.setDown(not self.touch_button.isDown())
        elif event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
            if event.key() == Qt.Key_I:
                self.buttons[0].setDown(False)
            elif event.key() == Qt.Key_Comma:
                self.buttons[1].setDown(False)
            elif event.key() == Qt.Key_J:
                self.buttons[2].setDown(False)
            elif event.key() == Qt.Key_L:
                self.buttons[3].setDown(False)
            elif event.key() == Qt.Key_K:
                self.buttons[4].setDown(False)
        return super(Handle, self).eventFilter(obj, event)

    def add_arguments(parser):
        group = parser.add_argument_group('Options for cabot_handle_simulator plugin')
        group.add_argument('--use_directional_indicator', action='store_true', help='Whether to use directional indicator')

    def shutdown_plugin(self):
        pass
