import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
import ruckig
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
        max_linear_vel: float = 0.8,
        max_angular_vel: float = 3.0,
        max_linear_acc: float = 6.0,
        max_angular_acc: float = 8.0,
        max_joint_vel: float = 5.0,
        min_linear_vel: float = 0.03,
        min_angular_vel: float = 0.1,
        linear_gain: float = 20.0,
        angular_gain: float = 12.0,
        max_joint_acc: float = 10.0,
        max_joint_jerk: float = 100.0,
    ):
        """
        Initialize the Pinocchio robot with servo control capabilities.
        """
        # Load robot model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.data_tmp = self.model.createData()

        # Get end-effector frame ID
        try:
            self.ee_frame_id = self.model.getFrameId(ee_link)
        except RuntimeError:
            print(f"Warning: Frame '{ee_link}' not found. Using frame 0.")
            self.ee_frame_id = 0

        # Robot state
        self.q = pin.neutral(self.model)  # Joint positions
        self.dq = np.zeros(self.model.nv)  # Joint velocities
        self.ddq = np.zeros(self.model.nv)  # Joint accelerations

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
        self.max_joint_acc = max_joint_acc
        self.max_joint_jerk = max_joint_jerk
        self.min_linear_vel = min_linear_vel
        self.min_angular_vel = min_angular_vel
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain

        # Control gains - REDUCED for stability
        self.kp_pos = 1.0  # Position gain (reduced from 5.0)
        self.kp_ori = 0.5  # Orientation gain (reduced from 3.0)
        self.kd = 0.05  # Damping gain
        self.damping = 1e-4  # Increased damping for stability

        # IK regularization parameters
        self.joint_regularization = 0.01  # Joint position regularization weight
        self.velocity_regularization = 0.001  # Joint velocity regularization weight
        self.manipulability_threshold = (
            0.01  # Threshold for manipulability-based regularization
        )
        self.desired_joint_config = pin.neutral(
            self.model
        ).copy()  # Target joint configuration for regularization

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

    def get_ee_pose(self, joint_positions_dict: dict = None) -> np.ndarray:
        """Get current end-effector pose as 4x4 transformation matrix."""
        if joint_positions_dict is not None:
            return self._get_ee_pose_given_positions(joint_positions_dict)
        self.__update_kinematics()
        se3_pose = self.data.oMf[self.ee_frame_id]
        return se3_to_matrix(se3_pose)

    def _get_ee_pose_given_positions(self, joint_positions_dict: dict) -> np.ndarray:
        joint_positions = np.zeros(self.model.nq)
        for joint_name, position in joint_positions_dict.items():
            joint_index = self.__get_joint_index(joint_name)
            if joint_index < 0 or joint_index >= self.model.nq:
                raise ValueError(f"Joint '{joint_name}' not found in model.")
            joint_positions[joint_index] = position

        pin.forwardKinematics(self.model, self.data_tmp, joint_positions)
        pin.framesForwardKinematics(self.model, self.data_tmp, joint_positions)
        se3_pose = self.data_tmp.oMf[self.ee_frame_id]
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

        # Use regularized pseudo-inverse
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
        linear_tol: float = 0.0005,
        angular_tol: float = 0.005,
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
                    self.min_linear_vel / linear_vel_norm
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

        # Compute joint velocities using regularized pseudo-inverse
        J = self.__compute_jacobian()

        # Use regularized pseudo-inverse with multiple regularization terms
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

    def servo_to_joint_positions(
        self,
        target_joint_positions,
        dt: float = 0.01,
        joint_tol: float = 1e-4,
        max_joint_vel: float = None,
        max_joint_acc: float = None,
        max_joint_jerk: float = None,
    ) -> bool:
        """
        Move one jerk-limited Ruckig step toward target joint positions.

        Args:
            target_joint_positions: Dict of joint names to positions, or a vector
                ordered like ``self.q``.
            dt: Time step for online trajectory generation.
            joint_tol: Joint-space convergence tolerance.
            max_joint_vel: Optional per-joint velocity cap. Defaults to the robot's
                configured ``max_joint_vel``.
            max_joint_acc: Optional per-joint acceleration cap. Defaults to the
                robot's configured ``max_joint_acc``.
            max_joint_jerk: Optional per-joint jerk cap. Defaults to the robot's
                configured ``max_joint_jerk``.

        Returns:
            bool: True if the target joint positions are reached after this step.
        """
        if dt <= 0:
            raise ValueError("dt must be greater than zero.")
        if joint_tol < 0:
            raise ValueError("joint_tol must be non-negative.")

        target_q = self.__target_joint_positions_to_vector(target_joint_positions)

        velocity_cap = self.max_joint_vel if max_joint_vel is None else max_joint_vel
        if velocity_cap <= 0:
            raise ValueError("max_joint_vel must be greater than zero.")

        acceleration_cap = (
            self.max_joint_acc if max_joint_acc is None else max_joint_acc
        )
        if acceleration_cap <= 0:
            raise ValueError("max_joint_acc must be greater than zero.")

        jerk_cap = self.max_joint_jerk if max_joint_jerk is None else max_joint_jerk
        if jerk_cap <= 0:
            raise ValueError("max_joint_jerk must be greater than zero.")

        self.__validate_joint_position_limits(target_q, joint_tol)

        max_velocity = self.__joint_velocity_limits(velocity_cap)
        dofs = len(self.q)
        otg = ruckig.Ruckig(dofs, dt)
        ruckig_input = ruckig.InputParameter(dofs)
        ruckig_output = ruckig.OutputParameter(dofs)

        ruckig_input.current_position = self.q.tolist()
        ruckig_input.current_velocity = self.dq.tolist()
        ruckig_input.current_acceleration = self.ddq.tolist()
        ruckig_input.target_position = target_q.tolist()
        ruckig_input.target_velocity = np.zeros(dofs).tolist()
        ruckig_input.target_acceleration = np.zeros(dofs).tolist()
        ruckig_input.max_velocity = max_velocity.tolist()
        ruckig_input.max_acceleration = np.full(dofs, acceleration_cap).tolist()
        ruckig_input.max_jerk = np.full(dofs, jerk_cap).tolist()

        try:
            result = otg.update(ruckig_input, ruckig_output)
        except Exception as error:
            raise ValueError("Failed to calculate Ruckig joint trajectory.") from error

        if result not in (ruckig.Result.Working, ruckig.Result.Finished):
            raise ValueError(f"Ruckig failed to calculate joint trajectory: {result}")

        self.q = np.asarray(ruckig_output.new_position, dtype=float)
        self.dq = np.asarray(ruckig_output.new_velocity, dtype=float)
        self.ddq = np.asarray(ruckig_output.new_acceleration, dtype=float)

        remaining_error = target_q - self.q
        return bool(np.all(np.abs(remaining_error) <= joint_tol))

    def update_state(self, joint_velocities: np.ndarray, dt: float = 0.01):
        """Update robot state with given joint velocities."""
        previous_dq = self.dq.copy()

        # Store velocities
        self.dq = joint_velocities.copy()
        if dt > 0:
            self.ddq = (self.dq - previous_dq) / dt

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
        """Print current robot status including IK diagnostics."""
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

    def __target_joint_positions_to_vector(self, target_joint_positions) -> np.ndarray:
        if isinstance(target_joint_positions, dict):
            target_q = self.q.copy()
            for joint_name, position in target_joint_positions.items():
                joint_index = self.__get_joint_index(joint_name)
                if joint_index < 0 or joint_index >= self.model.nq:
                    raise ValueError(f"Joint '{joint_name}' not found in model.")
                target_q[joint_index] = position
        else:
            target_q = np.asarray(target_joint_positions, dtype=float)
            if target_q.shape != self.q.shape:
                raise ValueError(
                    "target_joint_positions must have shape "
                    f"{self.q.shape}, got {target_q.shape}"
                )

        if not np.all(np.isfinite(target_q)):
            raise ValueError("target_joint_positions must be finite.")

        return target_q

    def __joint_velocity_limits(self, velocity_cap: float) -> np.ndarray:
        max_velocity = np.full(len(self.q), velocity_cap, dtype=float)
        for i in range(len(max_velocity)):
            if i < len(self.dq_max) and self.dq_max[i] > 0:
                max_velocity[i] = min(max_velocity[i], self.dq_max[i])
        return max_velocity

    def __validate_joint_position_limits(
        self, target_q: np.ndarray, joint_tol: float
    ) -> None:
        lower_limit_mask = np.isfinite(self.q_min)
        below_limits = (
            target_q[lower_limit_mask] < self.q_min[lower_limit_mask] - joint_tol
        )
        if np.any(below_limits):
            raise ValueError("target_joint_positions are below the robot joint limits.")

        upper_limit_mask = np.isfinite(self.q_max)
        above_limits = (
            target_q[upper_limit_mask] > self.q_max[upper_limit_mask] + joint_tol
        )
        if np.any(above_limits):
            raise ValueError("target_joint_positions are above the robot joint limits.")

    def __compute_regularized_jacobian_pinv(self, J: np.ndarray) -> np.ndarray:
        """
        Compute regularized pseudo-inverse of Jacobian with multiple regularization terms.

        Args:
            J: Jacobian matrix

        Returns:
            Regularized pseudo-inverse of Jacobian
        """
        n_joints = J.shape[1]

        # Compute manipulability measure
        manipulability = np.sqrt(np.linalg.det(J @ J.T))

        # Adaptive damping based on manipulability
        adaptive_damping = self.damping
        if manipulability < self.manipulability_threshold:
            # Increase damping near singularities
            damping_factor = self.manipulability_threshold / (manipulability + 1e-8)
            adaptive_damping = self.damping * min(damping_factor, 10.0)

        # Joint position regularization (bias towards desired joint configuration)
        joint_position_error = self.q - self.desired_joint_config
        W_joint = np.eye(n_joints) * self.joint_regularization

        # Joint velocity regularization (bias towards zero velocity)
        W_velocity = np.eye(n_joints) * self.velocity_regularization

        # Combined regularization matrix
        W_reg = W_joint + W_velocity

        # Damped least squares with regularization
        # J_pinv = (J^T J + λI + W_reg)^(-1) J^T
        JTJ = J.T @ J
        regularization_matrix = adaptive_damping * np.eye(n_joints) + W_reg

        try:
            # Primary method: Regularized normal equation
            J_pinv = np.linalg.solve(JTJ + regularization_matrix, J.T)
        except np.linalg.LinAlgError:
            # Fallback: SVD-based regularized pseudo-inverse
            U, s, Vt = np.linalg.svd(J, full_matrices=False)
            s_reg = s / (s**2 + adaptive_damping)  # Regularized singular values
            J_pinv = Vt.T @ np.diag(s_reg) @ U.T

        # Add joint position bias term
        joint_bias = -self.joint_regularization * joint_position_error

        return J_pinv, joint_bias

    def set_regularization_params(
        self,
        joint_regularization: float = None,
        velocity_regularization: float = None,
        manipulability_threshold: float = None,
        damping: float = None,
        desired_joint_config: np.ndarray = None,
    ):
        """
        Set IK regularization parameters.

        Args:
            joint_regularization: Weight for joint position regularization (bias towards desired config)
            velocity_regularization: Weight for joint velocity regularization (bias towards zero)
            manipulability_threshold: Threshold for manipulability-based adaptive damping
            damping: Base damping factor for singularity avoidance
            desired_joint_config: Target joint configuration for regularization (bias towards these angles)
        """
        if joint_regularization is not None:
            self.joint_regularization = joint_regularization
        if velocity_regularization is not None:
            self.velocity_regularization = velocity_regularization
        if manipulability_threshold is not None:
            self.manipulability_threshold = manipulability_threshold
        if damping is not None:
            self.damping = damping
        if desired_joint_config is not None:
            if len(desired_joint_config) != self.model.nq:
                raise ValueError(
                    f"desired_joint_config must have {self.model.nq} elements, got {len(desired_joint_config)}"
                )
            self.desired_joint_config = np.array(desired_joint_config)

        print(f"Regularization parameters updated:")
        print(f"  Joint regularization: {self.joint_regularization}")
        print(f"  Velocity regularization: {self.velocity_regularization}")
        print(f"  Manipulability threshold: {self.manipulability_threshold}")
        print(f"  Damping: {self.damping}")
        print(f"  Desired joint config: {np.round(self.desired_joint_config, 3)}")


if __name__ == "__main__":
    import time

    urdf_path = "./lite6.urdf"
    try:
        # Initialize robot
        robot = JacobiRobot(
            urdf_path, ee_link="link6", max_linear_vel=0.05, max_angular_vel=0.2
        )  # Reduced limits

        print(
            "EEF pose",
            robot.get_ee_pose(
                {
                    "joint1": 0.0,
                    "joint2": 0.0,
                    "joint3": 0.0,
                    "joint4": 0.0,
                    "joint5": 0.0,
                    "joint6": 0.0,
                }
            ),
        )

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
