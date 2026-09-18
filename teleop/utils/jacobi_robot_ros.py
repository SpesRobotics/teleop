import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from typing import Optional, List
from teleop.utils.jacobi_robot import JacobiRobot
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import tempfile


class JacobiRobotROS(JacobiRobot):
    def __init__(
        self,
        node: Node,
        robot_description_topic: str = "/robot_description",
        ee_link: str = "end_effector",
        joint_names: Optional[List[str]] = None,
        joint_state_topic: str = "/joint_states",
        position_command_topic: str = "/joint_trajectory",
        max_linear_vel: float = 0.4,
        max_angular_vel: float = 0.9,
        max_linear_acc: float = 3.0,
        max_angular_acc: float = 6.0,
        max_joint_vel: float = 5.0,
        max_joint_acc: float = 10.0,
        max_joint_jerk: float = 100.0,
        min_linear_vel: float = 0.03,
        min_angular_vel: float = 0.1,
        linear_gain: float = 5.0,
        angular_gain: float = 1.0,

    ):
        """
        Initialize ROS 2-enabled Jacobian robot.

        Args:
            node: ROS 2 node instance
            robot_description_topic: Topic for robot description (URDF)
            ee_link: End-effector frame name
            joint_names: List of joint names (if None, will be extracted from URDF)
            joint_state_topic: Topic for joint states
            position_command_topic: Topic for position commands
            velocity_command_topic: Topic for velocity commands
            max_linear_vel: Maximum linear velocity
            max_angular_vel: Maximum angular velocity
            max_linear_acc: Maximum linear acceleration
            max_angular_acc: Maximum angular acceleration
            max_joint_vel: Maximum joint velocity
            min_linear_vel: Minimum linear velocity
            min_angular_vel: Minimum angular velocity
            linear_gain: Linear gain for control
            angular_gain: Angular gain for control
            max_joint_acc: Maximum joint acceleration
            max_joint_jerk: Maximum joint jerk
        """

        self.node = node
        urdf_path = self.__get_robot_description_from_topic(robot_description_topic)

        # Initialize JacobiRobot
        super().__init__(
            urdf_path,
            ee_link,
            max_linear_vel,
            max_angular_vel,
            max_linear_acc,
            max_angular_acc,
            max_joint_vel,
            min_linear_vel,
            min_angular_vel,
            linear_gain,
            angular_gain,
            max_joint_acc,
            max_joint_jerk,
        )

        self.joint_states_received = False

        # Joint configuration
        if joint_names is None:
            self.joint_names = self.__extract_joint_names_from_urdf()
        else:
            self.joint_names = joint_names

        # ROS 2 subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState, joint_state_topic, self.__joint_state_callback, 1
        )
        self.trajectory_publisher = self.node.create_publisher(
            JointTrajectory, position_command_topic, 1
        )

    def __get_robot_description_from_topic(self, topic: str) -> str:
        # Initialize URDF path from robot description topic
        urdf = None

        def robot_description_callback(msg: String):
            nonlocal urdf
            urdf = msg.data

        robot_description_sub = self.node.create_subscription(
            String,
            topic,
            robot_description_callback,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        robot_description_sub
        self.node.get_logger().info(f"Waiting for robot description on topic: {topic}")
        while urdf is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if not rclpy.ok():
                self.node.get_logger().error("Failed to receive robot description")
                return None

        with tempfile.NamedTemporaryFile(
            delete=False, suffix=".urdf"
        ) as temp_urdf_file:
            temp_urdf_file.write(urdf.encode("utf-8"))
            urdf_path = temp_urdf_file.name
        self.node.get_logger().info(f"URDF saved to temporary file: {urdf_path}")
        return urdf_path

    def __extract_joint_names_from_urdf(self) -> List[str]:
        """Extract joint names from Pinocchio model."""

        # find only non-fixed joints up to end-effector
        joint_names = []
        for i in range(1, self.model.njoints):  # Skip universe joint
            joint_name = self.model.names[i]
            if joint_name == "universe":
                continue

            joint_names.append(joint_name)
        return joint_names

    def __joint_state_callback(self, msg: JointState):
        """Callback for joint state updates."""
        if self.joint_states_received:
            return

        if not msg.name or not msg.position:
            return

        # Update joint positions
        for i, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            try:
                self.set_joint_position(name, msg.position[i])
            except ValueError as e:
                self.node.get_logger().error(
                    f"Failed to set joint position for {name}: {e}"
                )
        self.joint_states_received = True

    def __send_joint_trajectory_topic(
        self,
        duration: float = 0.05,
    ) -> bool:
        """Send joint trajectory via topic (not action)."""
        if not self.trajectory_publisher:
            self.node.get_logger().error("Trajectory publisher not available")
            return False

        # Create trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        # End point
        end_point = JointTrajectoryPoint()
        end_point.velocities = [
            self.get_joint_velocity(name) for name in self.joint_names
        ]
        end_point.positions = [
            self.get_joint_position(name) for name in self.joint_names
        ]

        end_point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9)
        )
        traj_msg.points.append(end_point)

        # Publish trajectory
        self.trajectory_publisher.publish(traj_msg)
        return True

    def twist(self, linear: np.ndarray, angular: np.ndarray, dt: float = 0.1) -> bool:
        """Send twist command to the robot."""
        reached = super().twist(linear, angular, dt)
        if reached is None:
            self.node.get_logger().error("Failed to compute joint positions for twist")
            return False

        self.__send_joint_trajectory_topic(duration=dt)
        return reached

    def servo_to_pose(self, target_pose: np.ndarray, dt: float = 0.1) -> bool:
        reached = super().servo_to_pose(target_pose, dt)
        if reached is None:
            self.node.get_logger().error(
                "Failed to compute joint positions for target pose"
            )
            return False

        self.__send_joint_trajectory_topic(duration=dt)
        return reached

    def servo_to_joint_positions(
        self,
        target_joint_positions,
        dt: float = 0.1,
        joint_tol: float = 1e-4,
        max_joint_vel: float = None,
        max_joint_acc: float = None,
        max_joint_jerk: float = None,
    ) -> bool:
        reached = super().servo_to_joint_positions(
            target_joint_positions,
            dt=dt,
            joint_tol=joint_tol,
            max_joint_vel=max_joint_vel,
            max_joint_acc=max_joint_acc,
            max_joint_jerk=max_joint_jerk,
        )

        self.__send_joint_trajectory_topic(duration=dt)
        return reached

    def reset_joint_states(self, blocked: bool = True):
        """Reset the robot state."""
        self.joint_states_received = False
        if not blocked:
            return
        while not self.joint_states_received:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if not rclpy.ok():
                self.node.get_logger().error("Failed to reset robot state")
                return

    def are_joint_states_received(self) -> bool:
        """Check if joint states have been received."""
        return self.joint_states_received


# sudo apt install ros-jazzy-moveit-resources-panda-moveit-config
# ros2 launch moveit_resources_panda_moveit_config demo.launch.py
def main():
    """Example usage of JacobiRobotROS."""
    rclpy.init()

    try:
        # Create ROS 2 node
        node = Node("jacobian_robot_node")

        # Initialize robot with node reference
        robot = JacobiRobotROS(
            node=node,
            ee_link="panda_hand",
            joint_names=[
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ],
            position_command_topic="/panda_arm_controller/joint_trajectory",
        )

        node.get_logger().info("Waiting for joint states...")
        robot.reset_joint_states()
        node.get_logger().info("Joint states received, robot initialized.")

        # Get current pose
        current_pose = robot.get_ee_pose()
        node.get_logger().info(f"Current EE position: {current_pose[:3, 3]}")

        target_poses = [
            np.copy(current_pose),
            np.copy(current_pose),
            np.copy(current_pose),
        ]
        target_poses[0][:3, 3] += np.array([0.3, 0.0, 0.0])
        target_poses[1][:3, 3] += np.array([0.0, 0.0, -0.2])

        pose_index = 0
        while True:
            reached = robot.servo_to_pose(target_poses[pose_index], dt=0.03)
            if reached:
                node.get_logger().info("Target pose reached.")
                pose_index = (pose_index + 1) % len(target_poses)
            try:
                rclpy.spin_once(node, timeout_sec=0.03)
            except rclpy.executors.ExternalShutdownException:
                node.get_logger().info("Node shutdown, exiting loop")
                break
            except Exception as e:
                pass

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
