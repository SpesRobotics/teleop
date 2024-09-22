import ssl
import os
import math
import logging
from werkzeug.serving import ThreadedWSGIServer
from typing import Callable
from flask import Flask, send_from_directory, request
import transforms3d as t3d
import numpy as np


TF_RUB2FLU = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
THIS_DIR = os.path.dirname(os.path.realpath(__file__))


def are_close(a, b=None, lin_tol=1e-9, ang_tol=1e-9):
    """
    Check if two transformation matrices are close to each other within specified tolerances.

    Parameters:
        a (numpy.ndarray): The first transformation matrix.
        b (numpy.ndarray, optional): The second transformation matrix. If not provided, it defaults to the identity matrix.
        lin_tol (float, optional): The linear tolerance for closeness. Defaults to 1e-9.
        ang_tol (float, optional): The angular tolerance for closeness. Defaults to 1e-9.

    Returns:
        bool: True if the matrices are close, False otherwise.
    """
    if b is None:
        b = np.eye(4)
    d = np.linalg.inv(a) @ b
    if not np.allclose(d[:3, 3], np.zeros(3), atol=lin_tol):
        return False
    rpy = t3d.euler.mat2euler(d[:3, :3])
    return np.allclose(rpy, np.zeros(3), atol=ang_tol)


class Teleop:
    """
    Teleop class for controlling a robot remotely.

    Args:
        host (str, optional): The host IP address. Defaults to "0.0.0.0".
        port (int, optional): The port number. Defaults to 4443.
        ssl_context (ssl.SSLContext, optional): The SSL context for secure communication. Defaults to None.
    """
    def __init__(self, host="0.0.0.0", port=4443, ssl_context=None):
        self.__server = None
        self.__host = host
        self.__port = port
        self.__ssl_context = ssl_context

        self.__relative_pose_init = None
        self.__absolute_pose_init = None
        self.__previous_received_pose = None
        self.__callbacks = []
        self.__pose = np.eye(4)

        if self.__ssl_context is None:
            self.__ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS)
            self.__ssl_context.load_cert_chain(
                certfile=os.path.join(THIS_DIR, "cert.pem"),
                keyfile=os.path.join(THIS_DIR, "key.pem"),
            )

        self.__app = Flask(__name__)
        log = logging.getLogger("werkzeug")
        log.setLevel(logging.ERROR)
        self.__register_routes()

    def set_pose(self, pose: np.ndarray) -> None:
        """
        Set the current pose of the end-effector.

        Parameters:
        - pose (np.ndarray): A 4x4 transformation matrix representing the pose.
        """
        self.__pose = pose

    def subscribe(self, callback: Callable[[np.ndarray, dict], None]) -> None:
        """
        Subscribe to receive updates from the teleop module.

        Parameters:
            callback (Callable[[np.ndarray, dict]]): A callback function that will be called when pose updates are received.
                The callback function should take two arguments:
                    - np.ndarray: A 4x4 transformation matrix representing the end-effector target pose.
                    - dict: A dictionary containing additional information.
        """
        self.__callbacks.append(callback)

    def __notify_subscribers(self, pose, message):
        for callback in self.__callbacks:
            callback(pose, message)

    def __update(self, message):
        move = message["move"]
        position = message["position"]
        orientation = message["orientation"]
        reference_frame = message["reference_frame"]

        if reference_frame not in ["gripper", "base"]:
            raise ValueError(
                f'Unknown reference frame: {reference_frame} (should be "gripper" or "base")'
            )

        position = np.array([position["x"], position["y"], position["z"]])
        quat = np.array(
            [orientation["w"], orientation["x"], orientation["y"], orientation["z"]]
        )

        if not move:
            self.__relative_pose_init = None
            self.__absolute_pose_init = None
            self.__notify_subscribers(self.__pose, message)
            return

        received_pose_rub = t3d.affines.compose(
            position, t3d.quaternions.quat2mat(quat), [1, 1, 1]
        )
        received_pose = TF_RUB2FLU @ received_pose_rub
        received_pose[:3, :3] = received_pose[:3, :3] @ np.linalg.inv(
            TF_RUB2FLU[:3, :3]
        )

        # Pose jump protection
        if self.__previous_received_pose is not None:
            if not are_close(
                received_pose,
                self.__previous_received_pose,
                lin_tol=6e-2,
                ang_tol=math.radians(25),
            ):
                print("Pose jump is detected, resetting the pose")
                self.__relative_pose_init = None
                self.__previous_received_pose = received_pose
                return
        self.__previous_received_pose = received_pose

        # Accumulate the pose and publish
        if self.__relative_pose_init is None:
            self.__relative_pose_init = received_pose
            self.__absolute_pose_init = self.__pose
            self.__previous_received_pose = None

        relative_pose = np.linalg.inv(self.__relative_pose_init) @ received_pose
        if reference_frame == "gripper":
            self.__pose = self.__absolute_pose_init @ relative_pose
        else:
            self.__pose = np.eye(4)
            self.__pose[:3, 3] = self.__absolute_pose_init[:3, 3] + relative_pose[:3, 3]
            self.__pose[:3, :3] = (
                relative_pose[:3, :3] @ self.__absolute_pose_init[:3, :3]
            )

        # Notify the subscribers
        self.__notify_subscribers(self.__pose, message)

    def __register_routes(self):
        @self.__app.route("/<path:filename>")
        def serve_file(filename):
            return send_from_directory(THIS_DIR, filename)

        @self.__app.route("/pose", methods=["POST"])
        def pose():
            json_data = request.get_json()
            self.__update(json_data)
            return {"status": "ok"}

        @self.__app.route("/log", methods=["POST"])
        def log():
            json_data = request.get_json()
            print(json_data)
            return {"status": "ok"}

        @self.__app.route("/")
        def index():
            return send_from_directory(THIS_DIR, "index.html")

    def run(self) -> None:
        """
        Runs the teleop server. This method is blocking.
        """
        # self.__app.run(host=self.__host, port=self.__port, ssl_context=self.__ssl_context, use_reloader=False, debug=True)
        self.__server = ThreadedWSGIServer(app=self.__app, host=self.__host, port=self.__port, ssl_context=self.__ssl_context)
        self.__server.serve_forever()

    def stop(self) -> None:
        """
        Stops the teleop server.
        """
        self.__server.shutdown()

