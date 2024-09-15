import numpy as np
import math
import transforms3d as t3d


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

    def register_callback(self, callback):
        self.__callbacks.append(callback)

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
            if not are_close(received_pose, self.__previous_received_pose, lin_tol=3e-2, ang_tol=math.radians(15)):
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
        for callback in self.__callbacks:
            callback(self.__pose)


class ROSTeleopPublisher:
    def __init__(self, initial_pose=None):
        self.teleop = TeleopPublisher(initial_pose)
        self.teleop.register_callback(self.publish_pose)
        
        import rclpy
        from tf2_ros import TransformBroadcaster
        from geometry_msgs.msg import TransformStamped

        rclpy.init()

        self.tf_message = TransformStamped()
        self.node = rclpy.create_node('teleop_publisher')
        self.broadcaster = TransformBroadcaster(self.node)

    def update(self, message):
        self.teleop.update(message)

    def publish_pose(self, pose):
        self.tf_message.header.stamp = self.node.get_clock().now().to_msg()
        self.tf_message.header.frame_id = 'base_link'
        self.tf_message.child_frame_id = 'tcp'
        self.tf_message.transform.translation.x = pose[0, 3]
        self.tf_message.transform.translation.y = pose[1, 3]
        self.tf_message.transform.translation.z = pose[2, 3]
        quat = t3d.quaternions.mat2quat(pose[:3, :3])
        self.tf_message.transform.rotation.w = quat[0]
        self.tf_message.transform.rotation.x = quat[1]
        self.tf_message.transform.rotation.y = quat[2]
        self.tf_message.transform.rotation.z = quat[3]

        self.broadcaster.sendTransform(self.tf_message)
