#!/usr/bin/env python3
import time
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


ERROR_NAME_BY_CODE = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "PLAN_INVALIDATED",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


class EndEffectorPnPLoop(Node):
    """Alternates between two end-effector poses forever."""

    def __init__(self) -> None:
        super().__init__("send_trajectory_pnp")
        self.startup_ok = False

        self.declare_parameter("action_name", "/move_action")
        self.declare_parameter("group_name", "Catalyst_Manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "link_five")

        self.declare_parameter("pose_a_xyz", [0.10, -0.42, 0.18])
        self.declare_parameter("pose_b_xyz", [0.10, -0.34, 0.24])
        self.declare_parameter("pose_a_quat_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("pose_b_quat_xyzw", [0.0, 0.0, 0.0, 1.0])

        self.declare_parameter("position_tolerance", 0.03)
        self.declare_parameter("constrain_orientation", False)
        self.declare_parameter("orientation_tolerance", 0.05)
        self.declare_parameter("allowed_planning_time", 6.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("max_velocity_scaling_factor", 0.25)
        self.declare_parameter("max_acceleration_scaling_factor", 0.25)
        self.declare_parameter("loop_pause_sec", 0.0)
        self.declare_parameter("retry_on_failure", True)
        self.declare_parameter("wait_for_server_timeout_sec", 60.0)

        action_name = str(self.get_parameter("action_name").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.pose_a_xyz = [float(v) for v in self.get_parameter("pose_a_xyz").value]
        self.pose_b_xyz = [float(v) for v in self.get_parameter("pose_b_xyz").value]
        self.pose_a_quat_xyzw = [
            float(v) for v in self.get_parameter("pose_a_quat_xyzw").value
        ]
        self.pose_b_quat_xyzw = [
            float(v) for v in self.get_parameter("pose_b_quat_xyzw").value
        ]

        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.constrain_orientation = bool(
            self.get_parameter("constrain_orientation").value
        )
        self.orientation_tolerance = float(
            self.get_parameter("orientation_tolerance").value
        )
        self.allowed_planning_time = float(
            self.get_parameter("allowed_planning_time").value
        )
        self.num_planning_attempts = int(
            self.get_parameter("num_planning_attempts").value
        )
        self.max_velocity_scaling = float(
            self.get_parameter("max_velocity_scaling_factor").value
        )
        self.max_acceleration_scaling = float(
            self.get_parameter("max_acceleration_scaling_factor").value
        )
        self.loop_pause_sec = float(self.get_parameter("loop_pause_sec").value)
        self.retry_on_failure = bool(self.get_parameter("retry_on_failure").value)
        self.wait_for_server_timeout_sec = float(
            self.get_parameter("wait_for_server_timeout_sec").value
        )

        for param_name, values, expected_len in (
            ("pose_a_xyz", self.pose_a_xyz, 3),
            ("pose_b_xyz", self.pose_b_xyz, 3),
            ("pose_a_quat_xyzw", self.pose_a_quat_xyzw, 4),
            ("pose_b_quat_xyzw", self.pose_b_quat_xyzw, 4),
        ):
            if len(values) != expected_len:
                raise ValueError(
                    f"'{param_name}' must contain exactly {expected_len} values."
                )
        if self.position_tolerance <= 0.0:
            raise ValueError("'position_tolerance' must be > 0.")
        if self.orientation_tolerance <= 0.0:
            raise ValueError("'orientation_tolerance' must be > 0.")
        if self.allowed_planning_time <= 0.0:
            raise ValueError("'allowed_planning_time' must be > 0.")
        if self.num_planning_attempts <= 0:
            raise ValueError("'num_planning_attempts' must be > 0.")
        if self.loop_pause_sec < 0.0:
            raise ValueError("'loop_pause_sec' must be >= 0.")

        for label, xyz in (("pose_a_xyz", self.pose_a_xyz), ("pose_b_xyz", self.pose_b_xyz)):
            if xyz[1] > -0.15:
                self.get_logger().warn(
                    f"{label} has y close to zero/positive in base_link. "
                    "This robot's nominal reachable workspace is mostly at negative y."
                )

        self._client = ActionClient(self, MoveGroup, action_name)
        self.get_logger().info(
            f"Waiting for MoveGroup action server: {action_name} "
            f"(timeout={self.wait_for_server_timeout_sec:.1f}s)"
        )
        if not self._client.wait_for_server(timeout_sec=self.wait_for_server_timeout_sec):
            self.get_logger().error(
                "MoveGroup action server is not available. "
                "Start MoveIt first with: 'ros2 launch Catalyst pnp.launch.py'."
            )
            return

        self.startup_ok = True

    def _build_goal_constraints(
        self,
        target_xyz: Sequence[float],
        target_quat_xyzw: Sequence[float],
        position_tolerance: float,
        constrain_orientation: bool,
    ) -> Constraints:
        target_pose = Pose()
        target_pose.position.x = target_xyz[0]
        target_pose.position.y = target_xyz[1]
        target_pose.position.z = target_xyz[2]
        target_pose.orientation.x = target_quat_xyzw[0]
        target_pose.orientation.y = target_quat_xyzw[1]
        target_pose.orientation.z = target_quat_xyzw[2]
        target_pose.orientation.w = target_quat_xyzw[3]

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]

        region = BoundingVolume()
        region.primitives = [sphere]
        region.primitive_poses = [target_pose]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.ee_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position_constraint]
        if constrain_orientation:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.base_frame
            orientation_constraint.link_name = self.ee_link
            orientation_constraint.orientation = target_pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
            orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
            orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints = [orientation_constraint]
        return constraints

    @staticmethod
    def _relaxed_target_xyz(target_xyz: Sequence[float]) -> List[float]:
        return [
            max(-0.35, min(0.35, target_xyz[0])),
            min(target_xyz[1], -0.25),
            max(target_xyz[2], 0.10),
        ]

    def _send_goal_and_wait(
        self,
        label: str,
        target_xyz: Sequence[float],
        target_quat_xyzw: Sequence[float],
        relaxed: bool = False,
    ) -> bool:
        xyz = list(target_xyz) if not relaxed else self._relaxed_target_xyz(target_xyz)
        position_tolerance = (
            self.position_tolerance
            if not relaxed
            else max(0.06, self.position_tolerance * 1.8)
        )
        constrain_orientation = self.constrain_orientation if not relaxed else False
        allowed_planning_time = (
            self.allowed_planning_time
            if not relaxed
            else max(10.0, self.allowed_planning_time * 2.0)
        )
        num_planning_attempts = (
            self.num_planning_attempts
            if not relaxed
            else max(20, self.num_planning_attempts * 2)
        )

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.goal_constraints = [
            self._build_goal_constraints(
                xyz, target_quat_xyzw, position_tolerance, constrain_orientation
            )
        ]
        goal.request.allowed_planning_time = allowed_planning_time
        goal.request.num_planning_attempts = num_planning_attempts
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal.request.start_state.is_diff = True

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(
            f"Sending pose {label} ({'relaxed retry' if relaxed else 'nominal'}): "
            f"xyz={xyz}, quat_xyzw={list(target_quat_xyzw)}"
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("MoveGroup goal request did not return a goal handle.")
            return False
        if not goal_handle.accepted:
            self.get_logger().error(f"MoveGroup goal rejected for pose {label}.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        action_result = result_future.result()
        if action_result is None:
            self.get_logger().error(
                f"MoveGroup did not return a result for pose {label}."
            )
            return False

        error_code_val = action_result.result.error_code.val
        error_code_name = ERROR_NAME_BY_CODE.get(error_code_val, "UNKNOWN")
        if error_code_val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"Pose {label} reached successfully.")
            return True

        self.get_logger().error(
            f"Pose {label} failed with MoveItErrorCodes={error_code_val} "
            f"({error_code_name})."
        )
        retryable_errors = (
            MoveItErrorCodes.FAILURE,
            MoveItErrorCodes.NO_IK_SOLUTION,
            MoveItErrorCodes.PLANNING_FAILED,
            MoveItErrorCodes.TIMED_OUT,
        )
        if self.retry_on_failure and not relaxed and error_code_val in retryable_errors:
            self.get_logger().warn(
                f"Retrying pose {label} once with relaxed constraints."
            )
            return self._send_goal_and_wait(
                label, target_xyz, target_quat_xyzw, relaxed=True
            )
        return False

    def run(self) -> None:
        self.get_logger().info("Starting infinite pose loop: A <-> B")
        sequence: Tuple[Tuple[str, List[float], List[float]], ...] = (
            ("A", self.pose_a_xyz, self.pose_a_quat_xyzw),
            ("B", self.pose_b_xyz, self.pose_b_quat_xyzw),
        )
        index = 0
        while True:
            if not rclpy.ok():
                break
            label, target_xyz, target_quat_xyzw = sequence[index]
            success = self._send_goal_and_wait(label, target_xyz, target_quat_xyzw)
            if not success:
                self.get_logger().warn(
                    f"Continuing loop after failed move to pose {label}."
                )
            index = (index + 1) % len(sequence)
            if self.loop_pause_sec > 0.0:
                time.sleep(self.loop_pause_sec)


def main() -> None:
    rclpy.init()
    node = EndEffectorPnPLoop()
    if not node.startup_ok:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
