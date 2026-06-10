import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from typing import List, Tuple


def se3_to_matrix(se3_obj: pin.SE3) -> np.ndarray:
    """Convert Pinocchio SE3 to 4x4 transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = se3_obj.rotation
    T[:3, 3] = se3_obj.translation
    return T


def matrix_to_se3(T: np.ndarray) -> pin.SE3:
    """Convert 4x4 transformation matrix to Pinocchio SE3."""
    return pin.SE3(T[:3, :3], T[:3, 3])


def matrix_log(T: np.ndarray) -> np.ndarray:
    """Compute SE(3) logarithm of transformation matrix, returns 6D twist vector."""
    se3_obj = matrix_to_se3(T)
    log_se3 = pin.log(se3_obj)
    return log_se3.vector  # Returns 6D vector [linear, angular]


def matrix_inverse(T: np.ndarray) -> np.ndarray:
    """Compute inverse of 4x4 transformation matrix."""
    T_inv = np.eye(4)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


class JacobiRobot:
    def __init__(
        self,
        urdf_path: str,
        ee_link: str = "end_effector",
        max_linear_vel: float = 0.4,
        max_angular_vel: float = 1.8,
        max_linear_acc: float = 3.0,
        max_angular_acc: float = 6.0,
        max_joint_vel: float = 5.0,
        min_linear_vel: float = 0.03,
        min_angular_vel: float = 0.1,
        linear_gain: float = 20.0,
        angular_gain: float = 8.0,
    ):
        """
        Initialize the Pinocchio robot with servo control capabilities.
        """
        # Load robot model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # Get end-effector frame ID
        try:
            self.ee_frame_id = self.model.getFrameId(ee_link)
        except RuntimeError:
            print(f"Warning: Frame '{ee_link}' not found. Using frame 0.")
            self.ee_frame_id = 0

        # Robot state
        self.q = pin.neutral(self.model)  # Joint positions
        self.dq = np.zeros(self.model.nv)  # Joint velocities

        # Joint limits
        self.q_min = self.model.lowerPositionLimit
        self.q_max = self.model.upperPositionLimit
        self.dq_max = self.model.velocityLimit

        # Servo control parameters
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.max_linear_acc = max_linear_acc
        self.max_angular_acc = max_angular_acc
        self.max_joint_vel = max_joint_vel
        self.min_linear_vel = min_linear_vel
        self.min_angular_vel = min_angular_vel
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain

        # Control gains - REDUCED for stability
        self.kp_pos = 1.0  # Position gain (reduced from 5.0)
        self.kp_ori = 0.5  # Orientation gain (reduced from 3.0)
        self.kd = 0.05  # Damping gain

        # IK solver parameters
        self.eps = 1e-4
        self.max_iter = 100
        self.damping = 1e-4  # Increased damping for stability
        self.regularization_enabled = False
        self.joint_regularization = 0.0
        self.velocity_regularization = 0.0
        self.desired_joint_config = self.q.copy()

        # Previous velocities for acceleration limiting
        self.prev_linear_vel = np.zeros(3)
        self.prev_angular_vel = np.zeros(3)

        # Visualization
        self.fig = None
        self.ax = None
        self.visualizing = False

    def __update_kinematics(self):
        """Update all kinematic quantities."""
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.framesForwardKinematics(self.model, self.data, self.q)
        pin.computeJointJacobians(self.model, self.data, self.q)

    def get_ee_pose(self) -> np.ndarray:
        """Get current end-effector pose as 4x4 transformation matrix."""
        self.__update_kinematics()
        se3_pose = self.data.oMf[self.ee_frame_id]
        return se3_to_matrix(se3_pose)

    def get_link_pose(self, link_name) -> np.ndarray:
        """Get current link pose as 4x4 transformation matrix."""
        if link_name not in [f.name for f in self.model.frames]:
            raise ValueError(f"[ERROR] Frame '{link_name}' not found in the model.")
        self.__update_kinematics()
        frame_id = self.model.getFrameId(link_name)
        se3_pose = self.data.oMf[frame_id]
        return se3_to_matrix(se3_pose)

    def __get_ee_velocity(self) -> np.ndarray:
        """Get current end-effector velocity."""
        self.__update_kinematics()
        J = pin.getFrameJacobian(
            self.model, self.data, self.ee_frame_id, pin.ReferenceFrame.LOCAL
        )
        return J @ self.dq

    def __compute_jacobian(self) -> np.ndarray:
        """Compute end-effector Jacobian."""
        self.__update_kinematics()
        return pin.getFrameJacobian(
            self.model, self.data, self.ee_frame_id, pin.ReferenceFrame.LOCAL
        )

    def twist(
        self,
        linear_velocity: np.ndarray,
        angular_velocity_rpy: np.ndarray,
        dt: float = 0.01,
    ) -> bool:
        # Move the end-effector with a given twist (velocity and angular velocity).
        J = self.__compute_jacobian()
        # Create desired spatial velocity vector
        desired_spatial_vel = np.concatenate([linear_velocity, angular_velocity_rpy])
        J_pinv, joint_bias = self.__compute_regularized_jacobian_pinv(J)
        joint_velocities = J_pinv @ desired_spatial_vel + joint_bias
        # Apply joint velocity limits
        for i in range(len(joint_velocities)):
            if i < len(self.dq_max) and self.dq_max[i] > 0:
                joint_velocities[i] = np.clip(
                    joint_velocities[i], -self.dq_max[i], self.dq_max[i]
                )
            else:
                # Default velocity limit if not specified
                joint_velocities[i] = np.clip(joint_velocities[i], -2.0, 2.0)
        # Check for excessive velocities (safety)
        if np.max(np.abs(joint_velocities)) > self.max_joint_vel:
            print("Warning: Excessive joint velocities detected, stopping!")
            return False
        # Update robot state
        self.update_state(joint_velocities, dt=dt)
        return True

    def servo_to_pose(
        self,
        target_pose: np.ndarray,
        dt: float = 0.01,
        linear_tol: float = 0.005,
        angular_tol: float = 0.01,
    ) -> bool:
        """
        Compute joint velocities for servoing to target pose with velocity/acceleration limits.

        Args:
            target_pose: 4x4 transformation matrix representing desired pose
            dt: Time step for numerical integration
            linear_tol: Linear position tolerance
            angular_tol: Angular orientation tolerance

        Returns:
            bool: True if target pose is reached, False otherwise
        """
        # Get current pose
        current_pose = self.get_ee_pose()

        # Compute pose error in SE(3)
        error_T = matrix_inverse(current_pose) @ target_pose
        error_vector = matrix_log(error_T)

        # Extract linear and angular errors
        linear_error = error_vector[:3]
        angular_error = error_vector[3:]

        # Compute error magnitudes
        linear_error_norm = np.linalg.norm(linear_error)
        angular_error_norm = np.linalg.norm(angular_error)

        # Desired twist with adaptive proportional control
        desired_linear_vel = self.kp_pos * self.linear_gain * linear_error
        desired_angular_vel = self.kp_ori * self.angular_gain * angular_error

        # Add minimum velocity to prevent stalling near target
        if linear_error_norm > linear_tol:
            linear_vel_norm = np.linalg.norm(desired_linear_vel)
            if linear_vel_norm > 0 and linear_vel_norm < self.min_linear_vel:
                desired_linear_vel = desired_linear_vel * (
                    self.min_angular_vel / linear_vel_norm
                )

        if angular_error_norm > angular_tol:
            angular_vel_norm = np.linalg.norm(desired_angular_vel)
            if angular_vel_norm > 0 and angular_vel_norm < self.min_angular_vel:
                desired_angular_vel = desired_angular_vel * (
                    self.min_angular_vel / angular_vel_norm
                )

        # Apply velocity limits
        linear_vel_norm = np.linalg.norm(desired_linear_vel)
        if linear_vel_norm > self.max_linear_vel:
            desired_linear_vel = desired_linear_vel * (
                self.max_linear_vel / linear_vel_norm
            )

        angular_vel_norm = np.linalg.norm(desired_angular_vel)
        if angular_vel_norm > self.max_angular_vel:
            desired_angular_vel = desired_angular_vel * (
                self.max_angular_vel / angular_vel_norm
            )

        # More relaxed acceleration limits near target
        accel_factor = 1.0
        if linear_error_norm < 5 * linear_tol or angular_error_norm < 5 * angular_tol:
            accel_factor = 3.0  # Allow 3x more acceleration near target

        # Apply acceleration limits with adaptive factor
        if dt > 0:
            linear_acc = (desired_linear_vel - self.prev_linear_vel) / dt
            linear_acc_norm = np.linalg.norm(linear_acc)
            max_linear_acc = self.max_linear_acc * accel_factor
            if linear_acc_norm > max_linear_acc:
                desired_linear_vel = (
                    self.prev_linear_vel
                    + (linear_acc / linear_acc_norm) * max_linear_acc * dt
                )

            angular_acc = (desired_angular_vel - self.prev_angular_vel) / dt
            angular_acc_norm = np.linalg.norm(angular_acc)
            max_angular_acc = self.max_angular_acc * accel_factor
            if angular_acc_norm > max_angular_acc:
                desired_angular_vel = (
                    self.prev_angular_vel
                    + (angular_acc / angular_acc_norm) * max_angular_acc * dt
                )

        # Store for next iteration
        self.prev_linear_vel = desired_linear_vel.copy()
        self.prev_angular_vel = desired_angular_vel.copy()

        # Combine into desired spatial velocity
        desired_spatial_vel = np.concatenate([desired_linear_vel, desired_angular_vel])

        # Reduced damping near target to prevent over-damping
        damping_factor = self.kd
        if linear_error_norm < 3 * linear_tol and angular_error_norm < 3 * angular_tol:
            damping_factor *= 0.5  # Reduce damping by half near target

        # Add damping based on current velocity
        current_spatial_vel = self.__get_ee_velocity()
        desired_spatial_vel -= damping_factor * current_spatial_vel

        # Compute joint velocities using damped pseudo-inverse
        J = self.__compute_jacobian()

        J_pinv, joint_bias = self.__compute_regularized_jacobian_pinv(J)
        joint_velocities = J_pinv @ desired_spatial_vel + joint_bias

        # Apply joint velocity limits
        for i in range(len(joint_velocities)):
            if i < len(self.dq_max) and self.dq_max[i] > 0:
                joint_velocities[i] = np.clip(
                    joint_velocities[i], -self.dq_max[i], self.dq_max[i]
                )
            else:
                # Default velocity limit if not specified
                joint_velocities[i] = np.clip(joint_velocities[i], -2.0, 2.0)

        # Check for excessive velocities (safety)
        if np.max(np.abs(joint_velocities)) > self.max_joint_vel:
            print("Warning: Excessive joint velocities detected, stopping!")
            return False

        # Update robot state
        self.update_state(joint_velocities, dt)

        # Check convergence
        current_pose = self.get_ee_pose()
        position_error = np.linalg.norm(current_pose[:3, 3] - target_pose[:3, 3])
        orientation_error = np.linalg.norm(
            pin.log(matrix_inverse(current_pose) @ target_pose).vector[3:]
        )
        reached = position_error < linear_tol and orientation_error < angular_tol
        return reached

    def update_state(self, joint_velocities: np.ndarray, dt: float = 0.01):
        """Update robot state with given joint velocities."""
        # Store velocities
        self.dq = joint_velocities.copy()

        # Integrate to get new joint positions
        self.q = self.q + self.dq * dt

        # Apply joint limits
        self.q = np.clip(self.q, self.q_min, self.q_max)

    def __get_link_transforms(self) -> List[np.ndarray]:
        """Get transforms of all links as 4x4 matrices."""
        self.__update_kinematics()
        transforms = []
        for i in range(self.model.nframes):
            se3_transform = self.data.oMf[i]
            transforms.append(se3_to_matrix(se3_transform))
        return transforms

    def start_visualization(self):
        """Initialize visualization (call in main thread)."""
        plt.ion()
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.visualizing = True
        plt.show(block=False)

    def update_visualization(self):
        """Update the 3D visualization (call in main thread)."""
        if not self.visualizing or self.ax is None:
            return

        # Clear previous plot
        self.ax.clear()

        # Get link transforms
        transforms = self.__get_link_transforms()

        # Plot coordinate frames for each link
        colors = ["red", "green", "blue"]

        for i, transform in enumerate(transforms):
            pos = transform[:3, 3]
            rot = transform[:3, :3]

            # Plot coordinate frame (only for some frames to reduce clutter)
            if i % 3 == 0:  # Show every 3rd frame
                for j in range(3):
                    axis = rot[:, j] * 0.05  # Smaller scale factor
                    self.ax.quiver(
                        pos[0],
                        pos[1],
                        pos[2],
                        axis[0],
                        axis[1],
                        axis[2],
                        color=colors[j],
                        alpha=0.7,
                    )

        # Connect consecutive frames with lines (simplified kinematic chain)
        positions = [t[:3, 3] for t in transforms]
        if len(positions) > 1:
            # Only connect the main kinematic chain (skip auxiliary frames)
            main_chain_indices = list(
                range(0, min(10, len(positions)))
            )  # First 10 frames
            for i in range(len(main_chain_indices) - 1):
                idx1, idx2 = main_chain_indices[i], main_chain_indices[i + 1]
                p1, p2 = positions[idx1], positions[idx2]
                self.ax.plot(
                    [p1[0], p2[0]],
                    [p1[1], p2[1]],
                    [p1[2], p2[2]],
                    "k-",
                    linewidth=2,
                    alpha=0.8,
                )

        # Highlight end-effector
        if self.ee_frame_id < len(positions):
            ee_pos = positions[self.ee_frame_id]
            self.ax.scatter(ee_pos[0], ee_pos[1], ee_pos[2], s=100, c="red", marker="s")

        # Set labels and limits
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.set_title(f"Robot Visualization - EE Frame: {self.ee_frame_id}")

        # Adaptive limits based on robot pose
        all_positions = np.array(positions)
        if len(all_positions) > 0:
            center = np.mean(all_positions, axis=0)
            max_range = max(
                0.5, np.max(np.linalg.norm(all_positions - center, axis=1)) * 1.2
            )

            self.ax.set_xlim([center[0] - max_range, center[0] + max_range])
            self.ax.set_ylim([center[1] - max_range, center[1] + max_range])
            self.ax.set_zlim([max(0, center[2] - max_range), center[2] + max_range])

        plt.draw()
        plt.pause(0.001)

    def stop_visualization(self):
        """Stop the visualization."""
        self.visualizing = False
        if self.fig:
            plt.close(self.fig)

    def print_status(self):
        """Print current robot status."""
        ee_pose = self.get_ee_pose()

        print(f"\n--- Robot Status ---")
        print(f"Joint positions: {np.round(self.q, 3)}")
        print(f"Joint velocities: {np.round(self.dq, 3)}")
        print(f"End-effector position: {np.round(ee_pose[:3, 3], 3)}")

    def __get_joint_index(self, joint_name: str) -> int:
        for i in range(self.model.njoints):
            if self.model.names[i] == joint_name:
                return i - 1
        raise ValueError(f"Joint '{joint_name}' not found in model.")

    def get_joint_position(self, joint_name: str) -> float:
        """Get current joint position by name."""
        joint_index = self.__get_joint_index(joint_name)
        if joint_index < 0 or joint_index >= self.model.njoints:
            raise ValueError(f"Joint '{joint_name}' not found in model.")
        return self.q[joint_index]

    def set_joint_position(self, joint_name: str, position: float):
        """Set joint position by name."""
        joint_index = self.__get_joint_index(joint_name)
        if joint_index < 0 or joint_index >= self.model.njoints:
            raise ValueError(f"Joint '{joint_name}' not found in model.")
        print(f"Setting joint '{joint_name}' to position {position:.3f}")
        self.q[joint_index] = position

    def get_joint_names(self) -> List[str]:
        """Get list of joint names in the robot model."""
        joint_names = []
        for i in range(self.model.njoints):
            joint_name = self.model.names[i]
            if joint_name != "universe":
                joint_names.append(joint_name)
        return joint_names

    def get_joint_velocity(self, joint_name: str) -> float:
        """Get current joint velocity by name."""
        joint_index = self.__get_joint_index(joint_name)
        if joint_index < 0 or joint_index >= self.model.njoints:
            raise ValueError(f"Joint '{joint_name}' not found in model.")
        return self.dq[joint_index]

    def __compute_regularized_jacobian_pinv(
        self, J: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute the original or opt-in regularized Jacobian pseudoinverse."""
        U, singular_values, Vt = np.linalg.svd(J, full_matrices=False)

        if not self.regularization_enabled:
            singular_inverse = np.where(
                singular_values > 1e-6,
                1.0 / singular_values,
                0.0,
            )
            J_pinv = Vt.T @ np.diag(singular_inverse) @ U.T
            return J_pinv, np.zeros(J.shape[1])

        denominator = (
            singular_values**2
            + self.damping**2
            + self.velocity_regularization
        )
        regularized_inverse = singular_values / denominator
        J_pinv = Vt.T @ np.diag(regularized_inverse) @ U.T

        nullspace = np.eye(J.shape[1]) - J_pinv @ J
        joint_error = self.desired_joint_config - self.q
        joint_bias = nullspace @ (self.joint_regularization * joint_error)
        return J_pinv, joint_bias

    def set_regularization_params(
        self,
        joint_regularization: float = None,
        velocity_regularization: float = None,
        damping: float = None,
        desired_joint_config: np.ndarray = None,
    ):
        """Enable and configure IK damping and joint-continuity regularization."""
        for name, value in (
            ("joint_regularization", joint_regularization),
            ("velocity_regularization", velocity_regularization),
            ("damping", damping),
        ):
            if value is not None:
                if value < 0:
                    raise ValueError(f"{name} must be non-negative")
                setattr(self, name, float(value))

        if desired_joint_config is not None:
            desired_joint_config = np.asarray(desired_joint_config, dtype=float)
            if desired_joint_config.shape != self.q.shape:
                raise ValueError(
                    f"desired_joint_config must have shape {self.q.shape}, "
                    f"got {desired_joint_config.shape}"
                )
            self.desired_joint_config = desired_joint_config.copy()

        self.regularization_enabled = True

    def disable_regularization(self):
        """Restore the original SVD pseudoinverse behavior for this instance."""
        self.regularization_enabled = False


if __name__ == "__main__":
    import time

    urdf_path = "./lite6.urdf"
    try:
        # Initialize robot
        robot = JacobiRobot(
            urdf_path, ee_link="link6", max_linear_vel=0.05, max_angular_vel=0.2
        )  # Reduced limits

        # Start visualization
        robot.start_visualization()

        # Get current pose and define small movement
        current_pose = robot.get_ee_pose()
        print(f"Current EE position: {current_pose[:3, 3]}")

        target_pose = current_pose.copy()
        target_pose[:3, 3] += np.array([0.1, 0.0, 0.1])

        print("Starting servo control...")
        robot.print_status()

        # Servo control loop
        dt = 0.01  # 100Hz for better stability
        max_steps = 2000

        for step in range(max_steps):
            # Compute servo velocities
            reached = robot.servo_to_pose(target_pose, dt)
            if reached:
                print(f"Target pose reached at step {step}!")
                break

            # Update visualization
            if step % 10 == 0:  # Update every 10 steps
                robot.update_visualization()

            time.sleep(dt)

        robot.print_status()
        input("Press Enter to stop...")

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        if "robot" in locals():
            robot.stop_visualization()
