#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agx_arm_msgs.msg import GripperStatus


class DoubleArmFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__("double_arm_feedback_subscriber")

        self.left_joint_state = None
        self.right_joint_state = None
        self.left_gripper_status = None
        self.right_gripper_status = None

        self.create_subscription(
            JointState,
            "/left_arm/feedback/joint_states",
            self.left_joint_state_callback,
            10,
        )
        self.create_subscription(
            JointState,
            "/right_arm/feedback/joint_states",
            self.right_joint_state_callback,
            10,
        )
        self.create_subscription(
            GripperStatus,
            "/left_arm/feedback/gripper_status",
            self.left_gripper_status_callback,
            10,
        )
        self.create_subscription(
            GripperStatus,
            "/right_arm/feedback/gripper_status",
            self.right_gripper_status_callback,
            10,
        )

        self.get_logger().info("Subscribed to double arm joint and gripper feedback")

    def left_joint_state_callback(self, msg: JointState):
        self.left_joint_state = msg
        self.generate_motion_from_feedback()

    def right_joint_state_callback(self, msg: JointState):
        self.right_joint_state = msg
        self.generate_motion_from_feedback()

    def left_gripper_status_callback(self, msg: GripperStatus):
        self.left_gripper_status = msg
        self.generate_motion_from_feedback()

    def right_gripper_status_callback(self, msg: GripperStatus):
        self.right_gripper_status = msg
        self.generate_motion_from_feedback()

    def generate_motion_from_feedback(self):
        if self.left_joint_state is None or self.right_joint_state is None:
            return

        left_joints = dict(zip(self.left_joint_state.name, self.left_joint_state.position))
        right_joints = dict(zip(self.right_joint_state.name, self.right_joint_state.position))

        left_gripper_width = None
        right_gripper_width = None
        if self.left_gripper_status is not None:
            left_gripper_width = self.left_gripper_status.width
        if self.right_gripper_status is not None:
            right_gripper_width = self.right_gripper_status.width

        # Put your motion generation logic here. The current feedback is available as:
        # left_joints, right_joints, left_gripper_width, right_gripper_width.
        self.get_logger().debug(
            "Feedback ready: "
            f"left_joints={left_joints}, "
            f"right_joints={right_joints}, "
            f"left_gripper_width={left_gripper_width}, "
            f"right_gripper_width={right_gripper_width}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DoubleArmFeedbackSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
