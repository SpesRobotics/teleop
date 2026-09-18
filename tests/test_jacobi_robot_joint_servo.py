import unittest
from pathlib import Path

import numpy as np

try:
    from teleop.utils.jacobi_robot import JacobiRobot
except ImportError as error:
    JacobiRobot = None
    JACOBI_IMPORT_ERROR = error
else:
    JACOBI_IMPORT_ERROR = None


class TestJacobiRobotJointServo(unittest.TestCase):
    def setUp(self):
        if JacobiRobot is None:
            self.skipTest(
                f"JacobiRobot dependencies are unavailable: {JACOBI_IMPORT_ERROR}"
            )

        urdf_path = (
            Path(__file__).resolve().parents[1]
            / "teleop"
            / "utils"
            / "lite6.urdf"
        )
        self.robot = JacobiRobot(str(urdf_path), ee_link="link6")

    def test_servo_to_joint_positions_steps_toward_named_targets(self):
        target = {
            "joint1": 0.2,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
            "joint5": 0.0,
            "joint6": 0.0,
        }

        reached = self.robot.servo_to_joint_positions(
            target,
            dt=0.1,
            max_joint_vel=1.0,
            max_joint_acc=10.0,
            max_joint_jerk=100.0,
        )

        self.assertFalse(reached)
        self.assertGreater(self.robot.get_joint_position("joint1"), 0.0)
        self.assertLess(self.robot.get_joint_position("joint1"), 0.1)
        self.assertLessEqual(abs(self.robot.get_joint_velocity("joint1")), 1.0)
        self.assertLessEqual(abs(self.robot.ddq[0]), 10.0)

        for _ in range(10):
            reached = self.robot.servo_to_joint_positions(
                target,
                dt=0.1,
                max_joint_vel=1.0,
                max_joint_acc=10.0,
                max_joint_jerk=100.0,
            )
            if reached:
                break

        self.assertTrue(reached)
        self.assertAlmostEqual(self.robot.get_joint_position("joint1"), 0.2)

    def test_servo_to_joint_positions_accepts_vector_targets(self):
        target = np.zeros_like(self.robot.q)
        target[0] = 0.05

        reached = self.robot.servo_to_joint_positions(
            target,
            dt=0.1,
            max_joint_vel=1.0,
            max_joint_acc=10.0,
            max_joint_jerk=100.0,
        )

        for _ in range(10):
            if reached:
                break
            reached = self.robot.servo_to_joint_positions(
                target,
                dt=0.1,
                max_joint_vel=1.0,
                max_joint_acc=10.0,
                max_joint_jerk=100.0,
            )

        self.assertTrue(reached)
        self.assertAlmostEqual(self.robot.get_joint_position("joint1"), 0.05)


if __name__ == "__main__":
    unittest.main()
