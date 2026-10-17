#!/usr/bin/env python3
"""Live table of Vive controller buttons/axes for the left and right hand.

Subscribes to the libsurvive Joy topic and shows one table per hand. Which
serial is which hand comes from config/vive_devices.yaml.

Run on the host (needs a display):
    source /opt/ros/kilted/setup.bash
    python3 scripts/vive_button_gui.py [--config PATH] [--topic /libsurvive/joy]
"""

import argparse
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
import tkinter as tk

from vive_config import BUTTON_MAX, default_config_path, load_vive_config

# Indices follow libsurvive's enum SurviveButton / enum SurviveAxis (survive_types.h).
BUTTONS = [
    ('Trigger', 0),
    ('Trackpad', 1),
    ('Thumbstick', 2),
    ('System', 3),
    ('A', 4),
    ('B', 5),
    ('Menu', 6),
    ('Grip', 7),
]
AXES = [
    ('Trigger', 1),
    ('Trackpad X', 2),
    ('Trackpad Y', 3),
    ('Joystick X', 10),
    ('Joystick Y', 11),
    ('Grip force', 8),
    ('Trackpad force', 9),
    ('Index finger', 7),
    ('Middle finger', 4),
    ('Ring finger', 5),
    ('Pinky finger', 6),
]

ON_BG, OFF_BG = '#3cb371', '#e8e8e8'


def spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def load_serials(path):
    params = load_vive_config(path)
    return {'Left': params['left_serial'], 'Right': params['right_serial']}


class JoyListener(Node):
    """Keeps the latest Joy message per device serial."""

    def __init__(self, topic):
        super().__init__('vive_button_gui')
        self._lock = threading.Lock()
        self._latest = {}
        self.create_subscription(Joy, topic, self._on_joy, 10)

    def _on_joy(self, msg):
        with self._lock:
            self._latest[msg.header.frame_id] = (time.monotonic(), msg)

    def latest(self, serial):
        with self._lock:
            return self._latest.get(serial)


class HandTable(tk.Frame):
    """One hand: a status line, a button table and an axis table."""

    def __init__(self, master, side, serial):
        super().__init__(master, padx=12, pady=8)
        self.serial = serial
        tk.Label(self, text=f'{side}  ({serial})', font=('TkDefaultFont', 13, 'bold')).grid(
            row=0, column=0, columnspan=3, sticky='w')
        self.status = tk.Label(self, text='waiting for data…', fg='gray')
        self.status.grid(row=1, column=0, columnspan=3, sticky='w', pady=(0, 6))

        for col, title in enumerate(('Button', 'Pressed', 'Touched')):
            tk.Label(self, text=title, font=('TkDefaultFont', 10, 'bold')).grid(
                row=2, column=col, sticky='ew')
        self.button_cells = []
        for r, (name, _) in enumerate(BUTTONS, start=3):
            tk.Label(self, text=name, anchor='w', width=14).grid(row=r, column=0, sticky='w')
            cells = [self._cell(r, 1), self._cell(r, 2)]
            self.button_cells.append(cells)

        r0 = 3 + len(BUTTONS) + 1
        for col, title in enumerate(('Axis', 'Value')):
            tk.Label(self, text=title, font=('TkDefaultFont', 10, 'bold')).grid(
                row=r0, column=col, sticky='ew', pady=(10, 0))
        self.axis_cells = []
        for r, (name, _) in enumerate(AXES, start=r0 + 1):
            tk.Label(self, text=name, anchor='w', width=14).grid(row=r, column=0, sticky='w')
            self.axis_cells.append(self._cell(r, 1))

    def _cell(self, row, col):
        cell = tk.Label(self, text='0', width=8, bg=OFF_BG, relief='ridge', bd=1)
        cell.grid(row=row, column=col, padx=1, pady=1, sticky='ew')
        return cell

    def update_from(self, entry):
        if entry is None:
            return
        stamp, msg = entry
        age = time.monotonic() - stamp
        # The driver only publishes on input changes, so an idle controller is not an error.
        self.status.config(text=f'last input event {age:.1f}s ago', fg='black')

        def bit(i):
            return msg.buttons[i] if i < len(msg.buttons) else 0

        for (_, idx), (pressed, touched) in zip(BUTTONS, self.button_cells):
            for cell, value in ((pressed, bit(idx)), (touched, bit(idx + BUTTON_MAX))):
                cell.config(text=str(value), bg=ON_BG if value else OFF_BG)

        for (_, idx), cell in zip(AXES, self.axis_cells):
            value = msg.axes[idx] if idx < len(msg.axes) else 0.0
            cell.config(text=f'{value:+.2f}', bg=ON_BG if abs(value) > 0.05 else OFF_BG)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None, help='path to vive_devices.yaml')
    parser.add_argument('--topic', default='/libsurvive/joy')
    args, ros_args = parser.parse_known_args()

    config = args.config or default_config_path()
    serials = load_serials(config)

    rclpy.init(args=ros_args)
    node = JoyListener(args.topic)
    threading.Thread(target=spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    root.title('Vive controller buttons')
    tables = []
    for col, (side, serial) in enumerate(serials.items()):
        table = HandTable(root, side, serial)
        table.grid(row=0, column=col, sticky='n')
        tables.append(table)
    tk.Label(root, text=f'config: {config}   topic: {args.topic}', fg='gray').grid(
        row=1, column=0, columnspan=2, sticky='w', padx=12, pady=(0, 8))

    def refresh():
        for table in tables:
            table.update_from(node.latest(table.serial))
        root.after(50, refresh)

    refresh()
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
