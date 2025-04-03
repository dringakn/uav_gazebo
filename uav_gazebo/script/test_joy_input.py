#!/usr/bin/env python3
"""
Author: [Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>]

PS4 Joystick Reader for Drone Navigation (Configurable)

This module provides an object-oriented interface for reading events from a
PS4 joystick via the Linux joystick interface. The joystick device (default
"/dev/input/js0") is opened in binary mode and events are processed in a
separate thread. Both axis and button mappings are configurable by name.
Additionally, you can configure the scale factors for axis values and define
deadzones (i.e., thresholds below which axis values are treated as zero).

--------------------------------------------------------------
Default Configuration:
--------------------------------------------------------------
Axis Mapping (Name -> Index):
    left_stick_x   : 0
    left_stick_y   : 1
    L2             : 2
    right_stick_x  : 3
    right_stick_y  : 4
    R2             : 5

Button Mapping (Name -> Index):
    square         : 0
    x              : 1
    circle         : 2
    triangle       : 3
    L1             : 4
    R1             : 5
    share          : 6
    options        : 7
    L3             : 8
    R3             : 9
    PS             : 10
    touchpad       : 11

Axis Scale (Name -> Scale Factor):
    By default, all axes are scaled by 1.0.

Deadzone (Name -> Threshold):
    By default, all axes have a deadzone of 0.1.

--------------------------------------------------------------
Usage:
--------------------------------------------------------------
1. Instantiate the PS4Joystick class. Optionally, you can provide dictionaries
   to override the default axis/button mappings, axis scales, and deadzones.
2. Query the current axis or button states using get_axis(name) and get_button(name).
3. Use these inputs to generate control commands for your simulated drone.

"""

import os
import struct
import threading
import time

class PS4Joystick:
    """Reads PS4 joystick events from /dev/input/js0 with configurable mappings and scaling."""

    # Event type constants (from Linux joystick API)
    JS_EVENT_BUTTON = 0x01  # Button event
    JS_EVENT_AXIS   = 0x02  # Axis event
    JS_EVENT_INIT   = 0x80  # Initialization event flag

    def __init__(self, device="/dev/input/js0",
                 axis_map=None, button_map=None,
                 axis_scale=None, deadzone=None):
        """
        Initialize the joystick reader.

        Args:
            device (str): Path to the joystick device (default: "/dev/input/js0").
            axis_map (dict): Mapping of axis names to their indices.
            button_map (dict): Mapping of button names to their indices.
            axis_scale (dict): Scale factors for axis values (name -> factor).
            deadzone (dict): Deadzone thresholds for axis values (name -> threshold).
        """
        self.device = device
        self.fd = open(self.device, "rb")

        # Default mappings if not provided
        self.axis_map = axis_map or {
            "left_stick_x": 0,
            "left_stick_y": 1,
            "L2": 2,
            "right_stick_x": 3,
            "right_stick_y": 4,
            "R2": 5
        }
        self.button_map = button_map or {
            "square": 0,
            "x": 1,
            "circle": 2,
            "triangle": 3,
            "L1": 4,
            "R1": 5,
            "share": 6,
            "options": 7,
            "L3": 8,
            "R3": 9,
            "PS": 10,
            "touchpad": 11
        }
        self.axis_scale = axis_scale or {name: 1.0 for name in self.axis_map.keys()}
        self.deadzone = deadzone or {name: 0.1 for name in self.axis_map.keys()}

        # Create reverse mappings (index -> name)
        self._axis_index_to_name = {v: k for k, v in self.axis_map.items()}
        self._button_index_to_name = {v: k for k, v in self.button_map.items()}

        # Current states (by name)
        self.axis_states = {}    # e.g., {"left_stick_x": 0.0, ...}
        self.button_states = {}  # e.g., {"square": 0, ...}

        self.running = True
        # Start reading thread
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        """Continuously read events from the joystick device and update states."""
        event_format = "IhBB"  # time (unsigned int), value (short), type (unsigned char), number (unsigned char)
        event_size = struct.calcsize(event_format)

        while self.running:
            event = self.fd.read(event_size)
            if event:
                time_stamp, value, event_type, number = struct.unpack(event_format, event)
                # Mask out initialization flag
                event_type = event_type & ~self.JS_EVENT_INIT
                if event_type == self.JS_EVENT_BUTTON:
                    # Map button index to configured name (or use index as string)
                    name = self._button_index_to_name.get(number, str(number))
                    self.button_states[name] = value
                elif event_type == self.JS_EVENT_AXIS:
                    # Map axis index to name
                    axis_name = self._axis_index_to_name.get(number, str(number))
                    normalized_value = self.normalize_axis(value)
                    scale = self.axis_scale.get(axis_name, 1.0)
                    deadzone = self.deadzone.get(axis_name, 0.0)
                    scaled_value = normalized_value * scale
                    # Apply deadzone: if within threshold, set to zero.
                    if abs(scaled_value) < deadzone:
                        scaled_value = 0.0
                    self.axis_states[axis_name] = scaled_value
            else:
                time.sleep(0.01)

    def normalize_axis(self, value):
        """
        Normalize a raw axis value to the range [-1.0, 1.0].

        Args:
            value (int): Raw axis value.

        Returns:
            float: Normalized axis value.
        """
        return value / 32767.0

    def get_axis(self, name):
        """
        Get the current value for a named axis.

        Args:
            name (str): Axis name (e.g., "right_stick_x").

        Returns:
            float: Normalized and scaled value (default 0.0 if not available).
        """
        return self.axis_states.get(name, 0.0)

    def get_button(self, name):
        """
        Get the current state for a named button.

        Args:
            name (str): Button name (e.g., "square").

        Returns:
            int: Button state (0 or 1; default 0 if not available).
        """
        return self.button_states.get(name, 0)

    def stop(self):
        """Stop reading events and close the device."""
        self.running = False
        self.thread.join()
        self.fd.close()

if __name__ == "__main__":
    # Create a PS4Joystick instance with the default configuration.
    joystick = PS4Joystick("/dev/input/js0")
    
    try:
        print("Reading joystick events. Press Ctrl+C to exit.")
        while True:
            # Retrieve axis values by configured names.
            left_stick_x = joystick.get_axis("left_stick_x")
            left_stick_y = joystick.get_axis("left_stick_y")
            right_stick_x = joystick.get_axis("right_stick_x")
            right_stick_y = joystick.get_axis("right_stick_y")
            L2 = joystick.get_axis("L2")
            R2 = joystick.get_axis("R2")

            # Retrieve button states.
            square = joystick.get_button("square")
            x_button = joystick.get_button("x")
            circle = joystick.get_button("circle")
            triangle = joystick.get_button("triangle")

            # Print the current state.
            print(f"Axes -> Left Stick: ({left_stick_x:.2f}, {left_stick_y:.2f}), "
                  f"Right Stick: ({right_stick_x:.2f}, {right_stick_y:.2f}), "
                  f"L2: {L2:.2f}, R2: {R2:.2f}")
            print(f"Buttons -> square: {square}, x: {x_button}, circle: {circle}, triangle: {triangle}")
            print("-----")
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting joystick reader...")
    finally:
        joystick.stop()
