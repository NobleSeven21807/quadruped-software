"""Keyboard teleop that works on Windows, macOS and Linux, including over SSH.

A terminal only reports key presses, never releases. A held key shows up as
the OS's auto-repeat: one press, a pause (~0.25-0.7 s), then a fast stream of
repeats. A key counts as held while its repeats keep arriving and released
once they stop.
"""

import os
import sys
import time

from control_policies.gait import VelocityCommand

QUIT = object()

FORWARD_SPEED = 0.10  # m/s
TURN_RATE = 0.5  # rad/s

# Key -> (vx, vy, yaw_rate) it adds while held.
DRIVE_KEYS = {
    "w": (FORWARD_SPEED, 0.0, 0.0),
    "s": (-FORWARD_SPEED, 0.0, 0.0),
    "a": (0.0, 0.0, TURN_RATE),
    "d": (0.0, 0.0, -TURN_RATE),
}
STOP_KEY = " "
QUIT_KEYS = ("q", "\x03")  # q or Ctrl+C

# Longer than the OS auto-repeat delay, so a held key isn't dropped before it repeats.
FIRST_PRESS_TIMEOUT = 0.7  # s
# A few repeat intervals: how long after letting go the robot keeps moving.
REPEAT_TIMEOUT = 0.15  # s


class _WindowsKeys:
    def __enter__(self):
        import msvcrt

        self._msvcrt = msvcrt
        return self

    def __exit__(self, *exc):
        pass

    def read(self):
        """Every key pressed since the last call, without blocking."""
        keys = []
        while self._msvcrt.kbhit():
            key = self._msvcrt.getwch()
            if key in ("\x00", "\xe0"):  # arrow/function keys arrive as two codes
                self._msvcrt.getwch()
                continue
            keys.append(key)
        return keys


class _UnixKeys:
    def __enter__(self):
        import termios
        import tty

        self._termios = termios
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)  # keys arrive immediately and aren't echoed
        return self

    def __exit__(self, *exc):
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_settings)

    def read(self):
        """Every key pressed since the last call, without blocking."""
        import select

        keys = []
        while select.select([self._fd], [], [], 0)[0]:
            keys.extend(os.read(self._fd, 32).decode(errors="ignore"))
        return keys


def _terminal_keys():
    return _WindowsKeys() if os.name == "nt" else _UnixKeys()


class _HeldKey:
    def __init__(self, now):
        self.last_seen = now
        self.repeating = False

    def seen(self, now):
        self.last_seen = now
        self.repeating = True

    def is_held(self, now):
        timeout = REPEAT_TIMEOUT if self.repeating else FIRST_PRESS_TIMEOUT
        return now - self.last_seen < timeout


class KeyboardTeleop:
    """Turns held W/A/S/D keys into a VelocityCommand. Space stops, Q quits.

    Use as a context manager so the terminal is restored afterwards:

        with KeyboardTeleop() as teleop:
            command = teleop.poll()
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._terminal = _terminal_keys()
        self._held = {}

    def __enter__(self):
        self._terminal.__enter__()
        return self

    def __exit__(self, *exc):
        self._terminal.__exit__(*exc)

    def poll(self):
        """Current command from the keys held right now, or QUIT."""
        return self.handle_keys(self._terminal.read())

    def handle_keys(self, keys):
        now = self._clock()
        for key in keys:
            key = key.lower()
            if key in QUIT_KEYS:
                return QUIT
            if key == STOP_KEY:
                self._held.clear()
            elif key in DRIVE_KEYS:
                if key in self._held:
                    self._held[key].seen(now)
                else:
                    self._held[key] = _HeldKey(now)

        self._held = {key: held for key, held in self._held.items() if held.is_held(now)}
        return _command_for(self._held)


def _command_for(keys):
    vx = sum(DRIVE_KEYS[key][0] for key in keys)
    vy = sum(DRIVE_KEYS[key][1] for key in keys)
    yaw_rate = sum(DRIVE_KEYS[key][2] for key in keys)
    return VelocityCommand(vx, vy, yaw_rate)
