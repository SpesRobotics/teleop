import numpy as np
import math
import transforms3d as t3d


def ros2numpy(pose):
    import geometry_msgs.msg

    xyz = None
    q = None

    if isinstance(pose, geometry_msgs.msg.PoseStamped):
        pose = pose.pose
    elif isinstance(pose, geometry_msgs.msg.TransformStamped):
        pose = pose.transform

    if isinstance(pose, geometry_msgs.msg.Pose):
        xyz = [pose.position.x, pose.position.y, pose.position.z]
        q = [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]
    elif isinstance(pose, geometry_msgs.msg.Transform):
        xyz = [pose.translation.x, pose.translation.y, pose.translation.z]
        q = [pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z]
    else:
        raise ValueError("Unknown type")

    transform = t3d.affines.compose(xyz, t3d.quaternions.quat2mat(q), [1, 1, 1])
    return transform


TF_RUB2FLU = np.array([
    [0,  0, -1, 0],
    [-1, 0,  0, 0],
    [0,  1,  0, 0],
    [0,  0,  0, 1]
])


def are_close(a, b=None, lin_tol=1e-9, ang_tol=1e-9):
    if b is None:
        b = np.eye(4)
    d = np.linalg.inv(a) @ b
    if not np.allclose(d[:3, 3], np.zeros(3), atol=lin_tol):
        return False
    rpy = t3d.euler.mat2euler(d[:3, :3])
    return np.allclose(rpy, np.zeros(3), atol=ang_tol)


class TeleopPublisher:
    def __init__(self, initial_pose=None):
        self.__relative_pose_init = None
        self.__absolute_pose_init = None
        self.__previous_received_pose = None
        self.__callbacks = []
        self.__pose = None

        if initial_pose is None:
            initial_pose = np.eye(4)
        self.set_pose(initial_pose)        

    def set_pose(self, pose):
        self.__pose = pose

    def subscribe(self, callback):
        self.__callbacks.append(callback)

    def __notify_subscribers(self, pose, message):
        for callback in self.__callbacks:
            callback(pose, message)

    def update(self, message):
        move = message['move']
        position = message['position']
        orientation = message['orientation']
        reference_frame = message['reference_frame']

        if reference_frame not in ['gripper', 'base']:
            raise ValueError(f'Unknown reference frame: {reference_frame} (should be "gripper" or "base")')

        position = np.array([position['x'], position['y'], position['z']])
        quat = np.array([orientation['w'], orientation['x'], orientation['y'], orientation['z']])

        if not move:
            self.__relative_pose_init = None
            self.__absolute_pose_init = None
            self.__notify_subscribers(self.__pose, message)
            return
        
        received_pose_rub = t3d.affines.compose(
            position,
            t3d.quaternions.quat2mat(quat),
            [1, 1, 1]
        )
        received_pose = TF_RUB2FLU @ received_pose_rub
        received_pose[:3, :3] = received_pose[:3, :3] @ np.linalg.inv(TF_RUB2FLU[:3, :3])

        # Pose jump protection
        if self.__previous_received_pose is not None:
            if not are_close(received_pose, self.__previous_received_pose, lin_tol=6e-2, ang_tol=math.radians(25)):
                print('Pose jump is detected, resetting the pose')
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
        if reference_frame == 'gripper':
            self.__pose = self.__absolute_pose_init @ relative_pose
        else:
            self.__pose = np.eye(4)
            self.__pose[:3, 3] = self.__absolute_pose_init[:3, 3] + relative_pose[:3, 3]
            self.__pose[:3, :3] = relative_pose[:3, :3] @ self.__absolute_pose_init[:3, :3]
        
        # Notify the subscribers
        self.__notify_subscribers(self.__pose, message)


class ROSTeleopPublisher():
    def __init__(self, initial_pose=None):
        self.teleop = TeleopPublisher(initial_pose)
        self.teleop.subscribe(self.publish_pose)
        
        from geometry_msgs.msg import PoseStamped
        import rclpy

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('test_teleop')
        
        self.current_pose_subscriber = self.node.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            1)
        self.current_pose_subscriber

        self.publisher_speed_limiter = self.node.create_publisher(
            PoseStamped, '/target_frame_raw', 1)

        self.message = PoseStamped()
        self.is_set_init_pose = False
        self.current_robot_pose = None

    def update(self, message):
        self.teleop.update(message)

    def current_pose_callback(self, msg):
        print(msg)
        self.current_robot_pose = msg
        if self.is_set_init_pose:
            return
        self.teleop.set_pose(msg)
        self.is_set_init_pose = True
        
    def publish_pose(self, pose, params):
        try:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        except Exception as e:
            pass

        if self.current_robot_pose is None:
            return
        
        if not params['move']:
            pose  =ros2numpy(self.current_robot_pose.pose)
            print(pose)
            self.teleop.set_pose(pose)
            return
        
        self.message.header.stamp = self.node.get_clock().now().to_msg()
        self.message.header.frame_id = 'base_link'
        self.message.pose.position.x = pose[0, 3]
        self.message.pose.position.y = pose[1, 3]
        self.message.pose.position.z = pose[2, 3]
        quat = t3d.quaternions.mat2quat(pose[:3, :3])
        self.message.pose.orientation.w = quat[0]
        self.message.pose.orientation.x = quat[1]
        self.message.pose.orientation.y = quat[2]
        self.message.pose.orientation.z = quat[3]

        self.publisher_speed_limiter.publish(self.message)
