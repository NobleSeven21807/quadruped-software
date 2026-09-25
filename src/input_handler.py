import sys
import termios
import tty

from command_classes import MoveCommand

QUIT = object()


def handle_key(key):
    key = key.lower()
    if key == "w":
        return MoveCommand(MoveCommand.Direction.FORWARDS)
    if key == "s":
        return MoveCommand(MoveCommand.Direction.BACKWARDS)
    if key == "a":
        return MoveCommand(MoveCommand.Direction.ROTATE_LEFT)
    if key == "d":
        return MoveCommand(MoveCommand.Direction.ROTATE_RIGHT)
    return None


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def watch_inputs():
    key = get_key()
    if key.lower() == "q" or key == "\x03":
        return QUIT
    return handle_key(key)
