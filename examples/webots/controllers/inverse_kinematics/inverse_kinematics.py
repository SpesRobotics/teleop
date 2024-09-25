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
            if "joint" in link.name and "pen" not in link.name:
                motor = self.getDevice(link.name)
                motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(timestep)
                self.__motors.append(motor)

    def move_to_position(self, pose):
        position = pose[:3, 3]
        orientation = pose[:3, :3]
        initial_position = (
            [0] + [m.getPositionSensor().getValue() for m in self.__motors] + [0]
        )
        ik_results = self.__arm_chain.inverse_kinematics(
            position, initial_position=initial_position, target_orientation=orientation, orientation_mode="all"
        )
        for i in range(len(self.__motors)):
            self.__motors[i].setPosition(ik_results[i + 1])

    def move_joints(self, positions):
        for i in range(len(self.__motors)):
            self.__motors[i].setPosition(positions[i])

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

    robot.move_joints([0, -1.0, 1.8, -2.0, -1.3, 0.4])
    timestep = int(robot.getBasicTimeStep())
    for _ in range(100):
        robot.step(timestep) != -1

    current_pose = robot.get_current_pose()
    teleop.set_pose(current_pose)

    teleop.subscribe(on_teleop_callback)
    thread = threading.Thread(target=teleop.run)
    thread.start()

    while robot.step(timestep) != -1:
        if target_pose is not None:
            robot.move_to_position(target_pose)

    teleop.stop()
    thread.join()


if __name__ == "__main__":
    main()
