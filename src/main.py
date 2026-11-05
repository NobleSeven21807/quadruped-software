import argparse

from command_classes import ACTUATOR_ORDER, QuadrupedRobot
from input_handler import QUIT, KeyboardTeleop

CONTROL_DT = 0.01  # s per control tick (100 Hz)


def make_backend(name):
    """Where the joint angles go. Add the servo backend here once the hardware exists."""
    if name == "sim":
        from backends.mujoco_backend import MujocoBackend

        return MujocoBackend(ACTUATOR_ORDER, viewer=True)
    raise ValueError(f"unknown backend {name!r}")


def status_line(robot):
    c = robot.command
    line = f"vx={c.vx:+.2f} m/s  yaw={c.yaw_rate:+.2f} rad/s"
    if robot.problem_legs:
        line += f"  at joint limit: {', '.join(robot.problem_legs)}"
    return line


def main():
    parser = argparse.ArgumentParser(description="Drive the quadruped with the keyboard.")
    parser.add_argument("--backend", choices=["sim"], default="sim")
    args = parser.parse_args()

    robot = QuadrupedRobot(make_backend(args.backend))
    robot.stand()
    print("Hold W/S to walk, A/D to turn, Space to stop, Q to quit")
    print("(keys go to this terminal: click back here after the viewer opens)")

    last_status = ""
    try:
        with KeyboardTeleop() as teleop:
            while robot.is_running():
                command = teleop.poll()
                if command is QUIT:
                    break
                robot.move(command, CONTROL_DT)
                status = status_line(robot)
                if status != last_status:
                    # Pad only enough to cover the previous line, so narrow terminals don't wrap.
                    print(f"\r{status:<{len(last_status)}}", end="", flush=True)
                    last_status = status
    finally:
        print()
        robot.close()


if __name__ == "__main__":
    main()
