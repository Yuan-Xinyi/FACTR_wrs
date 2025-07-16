# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
# ---------------------------------------------------------------------------

import time
import numpy as np

import rclpy
from factr_teleop.factr_teleop import FACTRTeleop


class FACTRTeleopGravComp(FACTRTeleop):
    """
    This class demonstrates the gravity compensation and null-space regulation function of the 
    FACTR teleop leader arm. Communication between the leader arm and the follower Franka arm
    is not implemented in this example.
    """

    def __init__(self):
        super().__init__()

    def set_up_communication(self):
        pass

    def get_leader_gripper_feedback(self):
        pass

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        return 0.0

    def get_leader_arm_external_joint_torque(self):
        # Return zero torque for gravity compensation demo
        return np.zeros(self.num_arm_joints)

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        pass


def main(args=None):
    rclpy.init(args=args)
    factr_teleop_grav_comp = FACTRTeleopGravComp()

    try:
        while rclpy.ok():
            rclpy.spin(factr_teleop_grav_comp)
    except KeyboardInterrupt:
        factr_teleop_grav_comp.get_logger().info("Keyboard interrupt received. Shutting down...")
        factr_teleop_grav_comp.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()