from teleop import Teleop
import transforms3d as t3d
import argparse

try:
    import rclpy
    import geometry_msgs.msg
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from tf2_ros import TransformBroadcaster
except ImportError:
    raise ImportError(
        "ROS2 is not sourced. Please source ROS2 before running this script."
    )


def ros2numpy(pose):
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


def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host address")
    parser.add_argument("--port", type=int, default=4443, help="Port number")
    parser.add_argument(
        "--topic", type=str, default="target_frame", help="Topic for pose publishing"
    )

    args = parser.parse_args()

    teleop = Teleop(host=args.host, port=args.port)
    current_robot_pose_message = None
    pose_initiated = False
    node = rclpy.create_node("ros2_teleop")
    pose_publisher = node.create_publisher(PoseStamped, args.topic, 1)
    broadcaster = TransformBroadcaster(node)

    def ros_current_pose_callback(msg):
        nonlocal current_robot_pose_message
        current_robot_pose_message = msg

    def teleop_pose_callback(pose, params):
        nonlocal current_robot_pose_message
        nonlocal teleop
        nonlocal node
        nonlocal pose_publisher
        nonlocal pose_initiated

        tf_message = TransformStamped()
        tf_message.header.stamp = node.get_clock().now().to_msg()
        tf_message.header.frame_id = "base_link"
        tf_message.child_frame_id = "teleop_target"
        tf_message.transform.translation.x = pose[0, 3]
        tf_message.transform.translation.y = pose[1, 3]
        tf_message.transform.translation.z = pose[2, 3]
        quat = t3d.quaternions.mat2quat(pose[:3, :3])
        tf_message.transform.rotation.w = quat[0]
        tf_message.transform.rotation.x = quat[1]
        tf_message.transform.rotation.y = quat[2]
        tf_message.transform.rotation.z = quat[3]
        broadcaster.sendTransform(tf_message)

        try:
            rclpy.spin_once(node, timeout_sec=0.01)
        except Exception as e:
            pass

        if current_robot_pose_message is None:
            return

        if not params["move"] and not pose_initiated:
            current_robot_pose = ros2numpy(current_robot_pose_message.pose)
            teleop.set_pose(current_robot_pose)
            pose_initiated = True
            return

        message = PoseStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "link_base"
        message.pose.position.x = pose[0, 3]
        message.pose.position.y = pose[1, 3]
        message.pose.position.z = pose[2, 3]
        quat = t3d.quaternions.mat2quat(pose[:3, :3])
        message.pose.orientation.w = quat[0]
        message.pose.orientation.x = quat[1]
        message.pose.orientation.y = quat[2]
        message.pose.orientation.z = quat[3]
        pose_publisher.publish(message)

    current_pose_subscriber = node.create_subscription(
        PoseStamped, "/current_pose", ros_current_pose_callback, 1
    )

    print(f"Server start on the adress https://{args.host}:{args.port}")
    teleop.subscribe(teleop_pose_callback)
    teleop.run()


if __name__ == "__main__":
    main()
