import numpy as np
import transforms3d as t3d


TF_RUB2FLU = np.array([
    [0,  0, -1, 0],
    [-1, 0,  0, 0],
    [0,  1,  0, 0],
    [0,  0,  0, 1]
])


class TeleopPublisher:
    def __init__(self, initial_pose=None):
        if initial_pose is None:
            initial_pose = np.eye(4)
        self.pose = initial_pose
        self.relative_pose_init = None
        self.absolute_pose_init = None

    def publish_pose(self):
        position = self.pose[:3, 3]
        orientation = t3d.euler.mat2euler(self.pose[:3, :3])
        print(f'position: {position}, orientation: {orientation}')

    def update(self, message):
        move = message['move']
        position = message['position']
        orientation = message['orientation']

        position = np.array([position['x'], position['y'], position['z']])
        quat = np.array([orientation['w'], orientation['x'], orientation['y'], orientation['z']])

        if not move:
            self.relative_pose_init = None
            self.absolute_pose_init = None
            return
        
        received_pose_rub = t3d.affines.compose(
            position,
            t3d.quaternions.quat2mat(quat),
            [1, 1, 1]
        )
        # received_pose = TF_RUB2FLU @ received_pose_rub
        received_pose = received_pose_rub

        if True:
            print('x', received_pose_rub[0, 3], received_pose[0, 3])
            print('y', received_pose_rub[1, 3], received_pose[1, 3])
            print('z', received_pose_rub[2, 3], received_pose[2, 3])
            rpy = t3d.euler.mat2euler(received_pose[:3, :3])
            rpy_rub = t3d.euler.mat2euler(received_pose_rub[:3, :3])
            print('r', rpy_rub[0], rpy[0])
            print('p', rpy_rub[1], rpy[1])
            print('y', rpy_rub[2], rpy[2])

        if self.relative_pose_init is None:
            self.relative_pose_init = received_pose
            self.absolute_pose_init = self.pose

        relative_pose = np.linalg.inv(self.relative_pose_init) @ received_pose
        relative_pose = relative_pose
        self.pose = self.absolute_pose_init @ relative_pose
        self.publish_pose()


class ROSTeleopPublisher(TeleopPublisher):
    def __init__(self, initial_pose=None):
        super().__init__(initial_pose)
        
        # create tf broadcaster
        import rclpy
        from tf2_ros import TransformBroadcaster
        from geometry_msgs.msg import TransformStamped

        rclpy.init()

        self.tf_message = TransformStamped()
        self.node = rclpy.create_node('teleop_publisher')
        self.broadcaster = TransformBroadcaster(self.node)

    def publish_pose(self):
        self.tf_message.header.stamp = self.node.get_clock().now().to_msg()
        self.tf_message.header.frame_id = 'base_link'
        self.tf_message.child_frame_id = 'tcp'
        self.tf_message.transform.translation.x = self.pose[0, 3]
        self.tf_message.transform.translation.y = self.pose[1, 3]
        self.tf_message.transform.translation.z = self.pose[2, 3]
        quat = t3d.quaternions.mat2quat(self.pose[:3, :3])
        self.tf_message.transform.rotation.w = quat[0]
        self.tf_message.transform.rotation.x = quat[1]
        self.tf_message.transform.rotation.y = quat[2]
        self.tf_message.transform.rotation.z = quat[3]

        self.broadcaster.sendTransform(self.tf_message)
