from command_classes import QuadrupedRobot
from input_handler import QUIT, watch_inputs


def main():
    robot = QuadrupedRobot()
    print("Press W/A/S/D to move, Q to quit")
    while True:
        command = watch_inputs()
        if command is QUIT:
            break
        if command is None:
            continue
        robot.move(command)
        print(f"Robot direction: {robot.command.direction.value}")


if __name__ == "__main__":
    main()
