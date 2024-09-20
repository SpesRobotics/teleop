from teleop import Teleop
import transforms3d as t3d
import argparse

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped

    
except ImportError:
    raise ImportError(
        "ROS2 is not sourced. Please source ROS2 before running this script."
    )


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


class ROSTeleopPublisher():

    def __init__(self, host, port, topic):      
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('ros2_teleop')
        
        self.current_pose_subscriber = self.node.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            1)
        self.current_pose_subscriber

        self.publisher_pose = self.node.create_publisher(
            PoseStamped, topic, 1)

        self.message = PoseStamped()
        self.is_set_init_pose = False
        self.current_robot_pose = None

        self.teleop = Teleop(host=host, port=port)
        self.teleop.subscribe(self.publish_pose)
        self.teleop.run()

    def update(self, message):
        self.teleop.update(message)

    def current_pose_callback(self, msg):
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

        self.publisher_pose.publish(self.message)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='localhost', help='Host address')
    parser.add_argument('--port', type=int, default=4443, help='Port number')
    parser.add_argument('--topic', type=str, default='target_frame', help='Topic for pose publishing')

    args = parser.parse_args()
    print(f'Server start on the adress https://{args.host}:{args.port}')

    ROSTeleopPublisher(host=args.host, port=args.port, topic=args.topic)
