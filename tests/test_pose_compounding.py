import unittest
import threading
import requests
import time
from teleop import Teleop


def get_message():
    return {
        "move": False,
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        "reference_frame": "base",
    }


BASE_URL = "https://localhost:4443"


class TestPoseCompounding(unittest.TestCase):
    @classmethod
    def __callback(cls, pose, message):
        cls.__last_pose = pose
        cls.__last_message = message
        print(pose)

    @classmethod
    def setUpClass(cls):
        cls.__last_pose = None
        cls.__last_message = None

        cls.teleop = Teleop()
        cls.teleop.subscribe(cls.__callback)
        cls.thread = threading.Thread(target=cls.teleop.run)
        cls.thread.daemon = True
        cls.thread.start()

    def test_response(self):
        payload = get_message()
        response = requests.post(
            f"{BASE_URL}/pose", json=payload, verify=False, timeout=5
        )

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"status": "ok"})

    def test_single_position_update(self):
        payload = get_message()

        # The first message with `move==True` is used as a reference
        payload["move"] = True
        response = requests.post(
            f"{BASE_URL}/pose", json=payload, verify=False, timeout=5
        )
        self.assertEqual(response.status_code, 200)
        self.assertIsNotNone(self.__last_pose)
        self.assertIsNotNone(self.__last_message)

        # Move the phone up by 5cm (Y-axis)
        payload["move"] = True
        payload["position"]["y"] = 0.05
        response = requests.post(
            f"{BASE_URL}/pose", json=payload, verify=False, timeout=5
        )
        self.assertEqual(response.status_code, 200)

        # In total the result should be 5cm on the Z-axis because of the RUB -> FLU conversion
        self.assertEqual(self.__last_pose[2, 3], 0.05)

        # Move the phone up by another 5cm (Y-axis)
        payload["move"] = True
        payload["position"]["y"] = 0.1
        response = requests.post(
            f"{BASE_URL}/pose", json=payload, verify=False, timeout=5
        )
        self.assertEqual(response.status_code, 200)

        self.assertEqual(self.__last_pose[2, 3], 0.1)

    @classmethod
    def tearDownClass(cls):
        cls.teleop.stop()


if __name__ == "__main__":
    unittest.main()
