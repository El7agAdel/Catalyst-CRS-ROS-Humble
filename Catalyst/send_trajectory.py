#!/usr/bin/env python3
from typing import List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryActionSender(Node):
    """
    Alternates two joint-space goals on the controller action interface:
    A -> B -> A -> B ... with back-to-back execution.
    """

    def __init__(self) -> None:
        super().__init__("send_trajectory_action")

        # self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")
        # self.declare_parameter("joints", ["joint_one", "joint_two", "joint_three", "joint_four", "joint_five"])
        # self.declare_parameter("positions_a", [0.5, 0.2, -0.3, 0.0, 0.1])
        # self.declare_parameter("positions_b", [0.0, 0.0, 0.0, 0.0, 0.0])
        # self.declare_parameter("duration_sec", 0.5)


        # action_name = str(self.get_parameter("action_name").value)
        # self.joints: List[str] = list(self.get_parameter("joints").value)
        # self.positions_a: List[float] = [float(x) for x in self.get_parameter("positions_a").value]
        # self.positions_b: List[float] = [float(x) for x in self.get_parameter("positions_b").value]
        # self.duration_sec = float(self.get_parameter("duration_sec").value)



        action_name  = "/arm_controller/follow_joint_trajectory"
        self.joints = ["joint_one", "joint_two", "joint_three", "joint_four", "joint_five"]
        self.positions_a = [1.3198, -0.3263, -1.7183, -0.2981, 0.5391]
        self.positions_b = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.duration_sec = 0.3



        if len(self.joints) != len(self.positions_a):
            raise ValueError(
                f"'joints' length ({len(self.joints)}) must match 'positions_a' length "
                f"({len(self.positions_a)})."
            )
        
        if len(self.joints) != len(self.positions_b):
            raise ValueError(
                f"'joints' length ({len(self.joints)}) must match 'positions_b' length "
                f"({len(self.positions_b)})."
            )
        

        if self.duration_sec <= 0.0:
            raise ValueError("'duration_sec' must be > 0.")


        self._client = ActionClient(self, FollowJointTrajectory, action_name)
        self.get_logger().info(f"Waiting for action server: {action_name}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Action server not available: {action_name}")


        
        self._goal_in_flight = False
        self._send_a_next = True

        self.get_logger().info(
            "Alternating sender started: A <-> B "
            f"(trajectory_duration={self.duration_sec:.3f}s)"
        )
        self._send_next_goal()

    def _send_goal(self, positions: List[float], label: str) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(self.duration_sec)
        point.time_from_start.nanosec = int((self.duration_sec % 1.0) * 1e9)
        goal.trajectory.points.append(point)

        self.get_logger().info(
            f"Sending {label}: joints={self.joints}, positions={positions}, "
            f"duration={self.duration_sec:.3f}s"
        )
        self._goal_in_flight = True
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _send_next_goal(self) -> None:
        if self._goal_in_flight:
            return
        if self._send_a_next:
            self._send_goal(self.positions_a, "A")
        else:
            self._send_goal(self.positions_b, "B")
        self._send_a_next = not self._send_a_next

    def _feedback_cb(self, feedback_msg) -> None:
        _ = feedback_msg

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by controller.")
            self._goal_in_flight = False
            self._send_next_goal()
            return

        self.get_logger().info("Trajectory goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("Trajectory execution succeeded.")
        else:
            self.get_logger().error(
                f"Trajectory execution failed with error_code={result.error_code}"
            )
        self._goal_in_flight = False
        self._send_next_goal()


def main() -> None:
    rclpy.init()
    node = TrajectoryActionSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
