import os
import math
import socket
import logging
from typing import Callable, List
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import transforms3d as t3d
import numpy as np
import json

TF_RUB2FLU = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
THIS_DIR = os.path.dirname(os.path.realpath(__file__))


def get_local_ip():
    try:
        # Connect to an external address (doesn't actually send data)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # Google DNS as a dummy target
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        return f"Error: {e}"


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
    yaw = math.atan2(d[1, 0], d[0, 0])
    pitch = math.asin(-d[2, 0])
    roll = math.atan2(d[2, 1], d[2, 2])
    rpy = np.array([roll, pitch, yaw])
    return np.allclose(rpy, np.zeros(3), atol=ang_tol)


def slerp(q1, q2, t):
    """Spherical linear interpolation between two quaternions."""
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    dot = np.dot(q1, q2)

    # If the dot product is negative, use the shortest path
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Linear interpolation fallback for nearly identical quaternions
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(dot)
    theta = theta_0 * t

    q3 = q2 - q1 * dot
    q3 = q3 / np.linalg.norm(q3)

    return q1 * np.cos(theta) + q3 * np.sin(theta)


def interpolate_transforms(T1, T2, alpha):
    """
    Interpolate between two 4x4 transformation matrices using SLERP + linear translation.

    Args:
        T1 (np.ndarray): Start transform (4x4)
        T2 (np.ndarray): End transform (4x4)
        alpha (float): Interpolation factor [0, 1]

    Returns:
        np.ndarray: Interpolated transform (4x4)
    """
    assert T1.shape == (4, 4) and T2.shape == (4, 4)
    assert 0.0 <= alpha <= 1.0

    # Translation
    t1 = T1[:3, 3]
    t2 = T2[:3, 3]
    t_interp = (1 - alpha) * t1 + alpha * t2

    # Rotation
    R1 = T1[:3, :3]
    R2 = T2[:3, :3]
    q1 = t3d.quaternions.mat2quat(R1)
    q2 = t3d.quaternions.mat2quat(R2)

    # SLERP
    q_interp = slerp(q1, q2, alpha)
    R_interp = t3d.quaternions.quat2mat(q_interp)

    # Final transform
    T_interp = np.eye(4)
    T_interp[:3, :3] = R_interp
    T_interp[:3, 3] = t_interp

    return T_interp


class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                # Remove broken connections
                self.active_connections.remove(connection)


class Teleop:
    """
    Teleop class for controlling a robot remotely using FastAPI and WebSockets.

    Args:
        host (str, optional): The host IP address. Defaults to "0.0.0.0".
        port (int, optional): The port number. Defaults to 4443.
        frontend_dir (str, optional): The directory containing the frontend assets. Defaults to THIS_DIR.
    """

    def __init__(
        self,
        host="0.0.0.0",
        port=4443,
        natural_phone_orientation_euler=None,
        natural_phone_position=None,
        frontend_dir=None,
    ):
        self.__logger = logging.getLogger("teleop")
        self.__logger.setLevel(logging.INFO)
        self.__logger.addHandler(logging.StreamHandler())

        self.__host = host
        self.__port = port

        self.__relative_pose_init = None
        self.__absolute_pose_init = None
        self.__previous_received_pose = None
        self.__callbacks = []
        self.__pose = np.eye(4)

        if natural_phone_orientation_euler is None:
            natural_phone_orientation_euler = [0, math.radians(-45), 0]
        if natural_phone_position is None:
            natural_phone_position = [0, 0, 0]
        self.__natural_phone_pose = t3d.affines.compose(
            natural_phone_position,
            t3d.euler.euler2mat(*natural_phone_orientation_euler),
            [1, 1, 1],
        )

        if frontend_dir is None:
            frontend_dir = THIS_DIR
        self.__frontend_dir = frontend_dir

        self.__app = FastAPI()
        self.__manager = ConnectionManager()

        # Configure logging
        logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
        self.__setup_routes()

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
            callback (Callable[[np.ndarray, dict], None]): A callback function that will be called when pose updates are received.
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
        scale = message.get("scale", 1.0)

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
        received_pose = received_pose @ self.__natural_phone_pose

        # Pose jump protection
        if self.__previous_received_pose is not None:
            if not are_close(
                received_pose,
                self.__previous_received_pose,
                lin_tol=0.05,
                ang_tol=math.radians(35),
            ):
                self.__logger.warning("Pose jump detected, resetting the pose")
                self.__relative_pose_init = None
                self.__previous_received_pose = received_pose
                return
        self.__previous_received_pose = received_pose

        # Accumulate the pose and publish
        if self.__relative_pose_init is None:
            self.__relative_pose_init = received_pose
            self.__absolute_pose_init = self.__pose
            self.__previous_received_pose = None

        relative_position = received_pose[:3, 3] - self.__relative_pose_init[:3, 3]
        relative_orientation = received_pose[:3, :3] @ np.linalg.inv(
            self.__relative_pose_init[:3, :3]
        )

        if scale > 1.0:
            relative_position *= scale

        self.__pose = np.eye(4)
        self.__pose[:3, 3] = self.__absolute_pose_init[:3, 3] + relative_position
        self.__pose[:3, :3] = relative_orientation @ self.__absolute_pose_init[:3, :3]

        # Apply scale
        if scale < 1.0:
            self.__pose = interpolate_transforms(
                self.__absolute_pose_init, self.__pose, scale
            )

        # Notify the subscribers
        self.__notify_subscribers(self.__pose, message)

    def __setup_routes(self):
        # Mount static files directory
        assets_dir = os.path.join(self.__frontend_dir, "assets")
        self.__app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")

        @self.__app.get("/")
        async def index():
            self.__logger.debug("Serving the index.html file")
            return FileResponse(os.path.join(self.__frontend_dir, "index.html"))

        @self.__app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await self.__manager.connect(websocket)
            self.__logger.info("Client connected")

            try:
                while True:
                    data = await websocket.receive_text()
                    message = json.loads(data)

                    if message.get("type") == "pose":
                        self.__logger.debug(f"Received pose data: {message['data']}")
                        self.__update(message["data"])
                    elif message.get("type") == "log":
                        self.__logger.info(f"Received log message: {message['data']}")

            except WebSocketDisconnect:
                self.__manager.disconnect(websocket)
                self.__logger.info("Client disconnected")

    def run(self) -> None:
        """
        Runs the teleop server. This method is blocking.
        """
        self.__logger.info(f"Server started at {self.__host}:{self.__port}")
        self.__logger.info(
            f"The phone web app should be available at https://{get_local_ip()}:{self.__port}"
        )

        ssl_keyfile = os.path.join(THIS_DIR, "key.pem")
        ssl_certfile = os.path.join(THIS_DIR, "cert.pem")

        uvicorn.run(
            self.__app,
            host=self.__host,
            port=self.__port,
            ssl_keyfile=ssl_keyfile,
            ssl_certfile=ssl_certfile,
            log_level="warning",
        )

    def stop(self) -> None:
        """
        Stops the teleop server.
        """
        # FastAPI/uvicorn handles shutdown automatically
        pass
