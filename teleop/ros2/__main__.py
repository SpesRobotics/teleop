from teleop import Teleop
import transforms3d as t3d

try:
    import rclpy
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
except ImportError:
    raise ImportError(
        "ROS2 is not sourced. Please source ROS2 before running this script."
    )


def main():
    rclpy.init()
    node = rclpy.create_node("teleop")
    broadcaster = TransformBroadcaster(node)

    def callback(pose, message):
        if not message["move"]:
            # self.teleop.set_pose(current_robot_pose)
            return
        tf_message = TransformStamped()
        tf_message.header.stamp = node.get_clock().now().to_msg()
        tf_message.header.frame_id = "base_link"
        tf_message.child_frame_id = "tcp"
        tf_message.transform.translation.x = pose[0, 3]
        tf_message.transform.translation.y = pose[1, 3]
        tf_message.transform.translation.z = pose[2, 3]
        quat = t3d.quaternions.mat2quat(pose[:3, :3])
        tf_message.transform.rotation.w = quat[0]
        tf_message.transform.rotation.x = quat[1]
        tf_message.transform.rotation.y = quat[2]
        tf_message.transform.rotation.z = quat[3]
        broadcaster.sendTransform(tf_message)

    teleop = Teleop()
    teleop.subscribe(callback)
    teleop.run()


if __name__ == "__main__":
    main()
