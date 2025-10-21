# This file contains the methods to add to BlessedKeyboardControl for Cartesian control
# Copy these into the class

def get_current_ee_pose(self):
    """Get current end-effector pose using forward kinematics."""
    if not self.kinematics:
        return None

    # Get joint positions as numpy array (exclude gripper)
    joint_pos = np.array([self.current_positions[name] for name in JOINT_NAMES[:-1]])
    return self.kinematics.forward_kinematics(joint_pos)

def move_cartesian(self, delta_x=0, delta_y=0, delta_z=0):
    """Move end-effector in Cartesian space using IK."""
    if not self.kinematics:
        self.logger.warning("Kinematics not available for Cartesian control")
        return

    try:
        # Get current EE pose
        current_pose = self.get_current_ee_pose()

        # Apply delta to position
        desired_pose = current_pose.copy()
        desired_pose[0, 3] += delta_x
        desired_pose[1, 3] += delta_y
        desired_pose[2, 3] += delta_z

        # Get current joint positions
        current_joints = np.array([self.current_positions[name] for name in JOINT_NAMES[:-1]])

        # Solve IK
        new_joints = self.kinematics.inverse_kinematics(
            current_joint_pos=current_joints,
            desired_ee_pose=desired_pose,
            position_weight=1.0,
            orientation_weight=0.01
        )

        # Update joint positions (exclude gripper)
        for i, name in enumerate(JOINT_NAMES[:-1]):
            self.current_positions[name] = new_joints[i]

        # Send to robot
        self.send_action()

        # Update stored EE pose
        self.current_ee_pose = desired_pose

        return True
    except Exception as e:
        self.logger.error(f"Cartesian move failed: {e}", exc_info=True)
        return False
