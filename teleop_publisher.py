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
        self.relative_pose_init = None
        self.absolute_pose_init = None
        self.pose = None

        if initial_pose is None:
            initial_pose = np.eye(4)
        self.set_initial_pose(initial_pose)        

    def set_initial_pose(self, pose):
        self.pose = pose @ TF_RUB2FLU

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

        if self.relative_pose_init is None:
            self.relative_pose_init = received_pose_rub
            self.absolute_pose_init = self.pose

        relative_pose = np.linalg.inv(self.relative_pose_init) @ received_pose_rub
        self.pose = self.absolute_pose_init @ relative_pose
        self.publish_pose(self.pose @ np.linalg.inv(TF_RUB2FLU))


class ROSTeleopPublisher(TeleopPublisher):
    def __init__(self, initial_pose=None):
        super().__init__(initial_pose)
        
        import rclpy
        from tf2_ros import TransformBroadcaster
        from geometry_msgs.msg import TransformStamped

        rclpy.init()

        self.tf_message = TransformStamped()
        self.node = rclpy.create_node('teleop_publisher')
        self.broadcaster = TransformBroadcaster(self.node)

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
