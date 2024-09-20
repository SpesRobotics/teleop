import sys
import tempfile
import threading

try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit(
        'The "ikpy" Python module is not installed. '
        'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"'
    )

import math
from controller import Robot
from teleop import Teleop


class RobotArm(Robot):
    def __init__(self):
        super().__init__()
        self.__arm_chain = None
        self.__motors = None

        # Create the arm chain from the URDF
        filename = None
        with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as file:
            filename = file.name
            file.write(self.getUrdf().encode("utf-8"))
        self.__arm_chain = Chain.from_urdf_file(
            filename,
            active_links_mask=[False, True, True, True, True, True, True, False],
            symbolic=False,
        )

        # Initialize the arm motors and encoders.
        timestep = int(self.getBasicTimeStep())
        self.__motors = []
        for link in self.__arm_chain.links:
            if "motor" in link.name:
                motor = self.getDevice(link.name)
                motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(timestep)
                self.__motors.append(motor)

    def move_to_position(self, x, y, z):
        initial_position = (
            [0] + [m.getPositionSensor().getValue() for m in self.__motors] + [0]
        )
        ik_results = self.__arm_chain.inverse_kinematics(
            [x, y, z], max_iter=4, initial_position=initial_position
        )

        # Recalculate the inverse kinematics of the arm if necessary.
        position = self.__arm_chain.forward_kinematics(ik_results)
        squared_distance = (
            (position[0, 3] - x) ** 2
            + (position[1, 3] - y) ** 2
            + (position[2, 3] - z) ** 2
        )
        if math.sqrt(squared_distance) > 0.03:
            ik_results = self.__arm_chain.inverse_kinematics([x, y, z])

        for i in range(len(self.__motors)):
            self.__motors[i].setPosition(ik_results[i + 1])

    def get_current_pose(self):
        positions = (
            [0] + [m.getPositionSensor().getValue() for m in self.__motors] + [0]
        )
        return self.__arm_chain.forward_kinematics(positions)


def main():
    target_pose = None
    robot = RobotArm()
    teleop = Teleop()

    def on_teleop_callback(pose, message):
        nonlocal target_pose

        if message["move"]:
            target_pose = pose

    current_pose = robot.get_current_pose()
    teleop.set_pose(current_pose)

    teleop.subscribe(on_teleop_callback)
    thread = threading.Thread(target=teleop.run)
    thread.start()

    timestep = int(4 * robot.getBasicTimeStep())
    while robot.step(timestep) != -1:
        print(target_pose)
        if target_pose is not None:
            x = target_pose[0, 3]
            y = target_pose[1, 3]
            z = target_pose[2, 3]
            robot.move_to_position(x, y, z)

    thread.join()


if __name__ == "__main__":
    main()
