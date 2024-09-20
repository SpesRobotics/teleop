import numpy as np
import math
import transforms3d as t3d
from teleop import Teleop



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


class ROSTeleopPublisher():
    def __init__(self):
        print('----------')
        self.teleop = Teleop(host='192.168.2.168', port=5000)
        print('----------')
        
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
        self.teleop.run()

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
            pose = ros2numpy(self.current_robot_pose.pose)
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


if __name__=='__main__':
    ROSTeleopPublisher()