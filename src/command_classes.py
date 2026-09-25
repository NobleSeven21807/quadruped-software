from enum import Enum
from pathlib import Path
import sys

_policies_dir = Path(__file__).resolve().parent / "Control-Policies"
if str(_policies_dir) not in sys.path:
    sys.path.insert(0, str(_policies_dir))

import Classical


class MoveCommand:
    class Direction(Enum):
        FORWARDS = "forwards"
        BACKWARDS = "backwards"
        ROTATE_LEFT = "rotate_left"
        ROTATE_RIGHT = "rotate_right"

    def __init__(self, direction=None):
        self.direction = direction


class Leg:
    class Direction(Enum):
        FORWARDS = "forwards"
        BACKWARDS = "backwards"

    def __init__(self, direction=None):
        self.direction = direction

    def move_leg(self, direction):
        self.direction = direction
        if direction == Leg.Direction.FORWARDS:
            Classical.move_leg_forward()
        elif direction == Leg.Direction.BACKWARDS:
            Classical.move_leg_backwards()


class QuadrupedRobot:
    def __init__(self):
        self.front_left = Leg()
        self.front_right = Leg()
        self.back_left = Leg()
        self.back_right = Leg()
        self.command = None

    def move(self, command):
        self.command = command
        if command.direction == MoveCommand.Direction.FORWARDS:
            self._set_all_legs(Leg.Direction.FORWARDS)
        elif command.direction == MoveCommand.Direction.BACKWARDS:
            self._set_all_legs(Leg.Direction.BACKWARDS)
        elif command.direction == MoveCommand.Direction.ROTATE_RIGHT:
            self.front_left.move_leg(Leg.Direction.FORWARDS)
            self.back_left.move_leg(Leg.Direction.FORWARDS)
            self.front_right.move_leg(Leg.Direction.BACKWARDS)
            self.back_right.move_leg(Leg.Direction.BACKWARDS)
        elif command.direction == MoveCommand.Direction.ROTATE_LEFT:
            self.front_left.move_leg(Leg.Direction.BACKWARDS)
            self.back_left.move_leg(Leg.Direction.BACKWARDS)
            self.front_right.move_leg(Leg.Direction.FORWARDS)
            self.back_right.move_leg(Leg.Direction.FORWARDS)

    def _set_all_legs(self, direction):
        self.front_left.move_leg(direction)
        self.front_right.move_leg(direction)
        self.back_left.move_leg(direction)
        self.back_right.move_leg(direction)
